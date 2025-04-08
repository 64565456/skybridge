#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.h"
#include "crc/crc32.h"
#include "com_uart.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "dfu.h"

#define LOG_TAG "DFU"

#define FW_HEADER_SIZE  64
#define DFU_MAGIC       0x1A2B3C4D

typedef struct
{
    const char *name;
    dfu_type_t type;
    uint32_t target_address;
    esp_partition_type_t partition_type;
    esp_partition_subtype_t partition_subtype;
} partition_desc_t;

static const esp_partition_type_t DFU_PARTITION_TYPE_FW = (esp_partition_type_t)0x40;
static const esp_partition_subtype_t DFU_PARTITION_SUBTYPE_FW_APP = (esp_partition_subtype_t)0x00;
static const partition_desc_t partition_descs[] =
{
    { "app",  DFU_TYPE_APP,  0x08010000, DFU_PARTITION_TYPE_FW, DFU_PARTITION_SUBTYPE_FW_APP  },
};

void dfu_enter_bootloader(void)
{
    // extern void prf_trace_shell_send(const char *cmd);
    // prf_trace_shell_send("reboot\n");

    extern void prf_ctrl_misc_set_reboot(bool force, bool trap_boot);
    prf_ctrl_misc_set_reboot(false, false);
}

void dfu_exit_bootloader(void)
{
    dfu_reboot_system();
}

bool dfu_verify(uint32_t mask)
{
    const uint16_t blksize = 4096;

    uint8_t *blkbuf = malloc(blksize);
    if (blkbuf == NULL)
    {
        ESP_LOGW(LOG_TAG, "blkbuf malloc failed");
        return false;
    }

    bool match = true;
    for (int i = 0; i < ARRAY_SIZE(partition_descs); i++)
    {
        const partition_desc_t *desc = &partition_descs[i];
        if ((mask & (1 << desc->type)) == 0) {
            ESP_LOGI(LOG_TAG, "skip %s", desc->name);
            continue;
        }

        const esp_partition_t *partition = esp_partition_find_first(desc->partition_type, desc->partition_subtype, NULL);
        if (partition == NULL) {
            ESP_LOGW(LOG_TAG, "partition %s not found", desc->name);
            continue;
        }

        uint8_t args[FW_HEADER_SIZE], *pargs = args;
        esp_partition_read(partition, 0, args, sizeof(args));

        uint32_t magic = get_u32_inc(&pargs);
        if (magic != DFU_MAGIC) {
            ESP_LOGW(LOG_TAG, "%s magic mismatch 0x%08lx", desc->name, magic);
            continue;
        }

        uint32_t fw_size = get_u32_inc(&pargs);
        uint32_t fw_crc = get_u32_inc(&pargs);
        ESP_LOGI(LOG_TAG, "verify %s size %lu crc 0x%08lx", desc->name, fw_size, fw_crc);

        uint32_t ccrc = 0xffffffff;
        uint32_t rsize = 0;
        do
        {
            uint32_t tsize = fw_size - rsize < blksize ? fw_size - rsize : blksize;
            esp_partition_read(partition, FW_HEADER_SIZE + rsize, blkbuf, tsize);
            ccrc = crc32_append(ccrc, (char *)blkbuf, tsize);
            rsize += tsize;
        } while (rsize < fw_size);
        ccrc ^= 0xffffffff;
        if (ccrc != fw_crc)
        {
            ESP_LOGW(LOG_TAG, "%s crc mismatch 0x%08lx", desc->name, ccrc);
            match = false;
        }
    }

    free(blkbuf);

    return match;
}

bool dfu_start(uint32_t mask)
{
    bool result = false;
    bool cmdok = false;

    uint8_t blver[2];
    uint16_t blksize;

    cmdok = dfu_get_bootloader_version(blver);
    CHECK_RETX(cmdok, false);
    ESP_LOGI(LOG_TAG, "bootloader version %u.%u", blver[0], blver[1]);
    vTaskDelay(pdMS_TO_TICKS(100));

    cmdok = dfu_get_mtu_size(&blksize);
    CHECK_RETX(cmdok, false);
    ESP_LOGI(LOG_TAG, "mtu size %u", blksize);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t *blkbuf = malloc(blksize);
    if (blkbuf == NULL)
    {
        ESP_LOGW(LOG_TAG, "blkbuf malloc failed");
        return false;
    }

    for (int i = 0; i < ARRAY_SIZE(partition_descs); i++)
    {
        const partition_desc_t *desc = &partition_descs[i];
        if ((mask & (1 << desc->type)) == 0) {
            ESP_LOGI(LOG_TAG, "skip %s", desc->name);
            continue;
        }

        const esp_partition_t *partition = esp_partition_find_first(desc->partition_type, desc->partition_subtype, NULL);
        if (partition == NULL) {
            ESP_LOGW(LOG_TAG, "partition %s not found", desc->name);
            break;
        }

        uint8_t args[FW_HEADER_SIZE], *pargs = args;
        esp_partition_read(partition, 0, args, sizeof(args));

        uint32_t magic = get_u32_inc(&pargs);
        if (magic != DFU_MAGIC) {
            ESP_LOGW(LOG_TAG, "magic mismatch 0x%08lx", magic);
            break;
        }

        uint32_t fw_addr = desc->target_address;
        uint32_t fw_size = get_u32_inc(&pargs);
        uint32_t fw_crc = get_u32_inc(&pargs);
        ESP_LOGI(LOG_TAG, "flash %s size %lu", desc->name, fw_size);

        cmdok = dfu_erase_flash(fw_addr, fw_size);
        if (cmdok) {
            ESP_LOGI(LOG_TAG, "erase %s ok", desc->name);
        } else {
            ESP_LOGW(LOG_TAG, "erase %s failed", desc->name);
            break;
        }

        uint32_t wsize = 0;
        do
        {
            uint32_t tsize = fw_size - wsize < blksize ? fw_size - wsize : blksize;
            esp_partition_read(partition, FW_HEADER_SIZE + wsize, blkbuf, tsize);

            float percent = (float)wsize / (float)fw_size * 100.0f;
            ESP_LOGI(LOG_TAG, "flashing %.2f%% %lu/%lu", percent, wsize, fw_size);
            cmdok = dfu_write_flash(fw_addr + wsize, blkbuf, tsize);
            CHECK_GO(cmdok, err);
            wsize += tsize;
        } while (wsize < fw_size);

        cmdok = dfu_verify_flash(fw_addr, fw_size, fw_crc);
        if (cmdok) {
            ESP_LOGI(LOG_TAG, "verify %s ok", desc->name);
        } else {
            ESP_LOGW(LOG_TAG, "verify %s failed", desc->name);
            break;
        }
    }

    result = cmdok;

err:
    free(blkbuf);
    return result;
}
