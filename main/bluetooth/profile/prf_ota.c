#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "main.h"
#include "dfu.h"
#include "com_uart.h"
#include "esp_log.h"
#include "crc/crc16.h"
#include "crc/crc32.h"
#include "svc_ota.h"
#include "esp_ota_ops.h"

#define LOG_TAG "OTA"

#define OTA_PACKET_SIZE     4096
#define OTA_QUEUE_NUM       16
#define OTA_TIMEOUT         3000

#define FW_MAGIC            0x1A2B3C4D
#define FW_HEADER_SIZE      64

static const esp_partition_type_t OTA_PARTITION_TYPE_FW = (esp_partition_type_t)0x40;
static const esp_partition_subtype_t OTA_PARTITION_SUBTYPE_FW_APP = (esp_partition_subtype_t)0x00;

typedef enum
{
    OTA_TYPE_ESP,
    OTA_TYPE_FW_APP,
} ota_type_t;

typedef enum
{
    OTA_CMD_START,      /* Master: type,size,crc32 */
    OTA_CMD_DATA,       /* Slave:  offset,size */
                        /* Master: offset,size,crc16,payload */
    OTA_CMD_FINISH,     /* Master: */
} ota_cmd_t;

typedef enum
{
    OTA_OK,
    OTA_ERR_CMD,
    OTA_ERR_PARAM,
    OTA_ERR_LENGTH,
    OTA_ERR_CRC,
    OTA_ERR_TIMEOUT,
    OTA_ERR_ABORT,
    OTA_ERR_UNKNOWN = 0xff,
} ota_err_t;

typedef enum
{
    OTA_STAGE_IDLE,
    OTA_STAGE_START,
    OTA_STAGE_DATA,
    OTA_STAGE_FINISH,
} ota_stage_t;

typedef void (*ota_msg_func_t)(uint8_t *data, uint16_t length);
typedef struct
{
    ota_msg_func_t func;
    uint32_t size;
    uint8_t data[];
} ota_msg_t;

static QueueHandle_t ota_msg_queue;
static TimerHandle_t ota_timeout_timer;
static TimerHandle_t ota_finish_timer;
static TaskHandle_t ota_task_handle;
static ota_stage_t ota_stage;
static uint8_t package_type;
static uint32_t package_size, package_received, package_crc;
static uint8_t packet_data[OTA_PACKET_SIZE];
static uint32_t packet_offset_request, packet_size_request, packet_received, packet_retry;
static uint16_t packet_crc;
static esp_ota_handle_t update_handle;
static const esp_partition_t *update_partition;

static void ota_data_handler(uint8_t *data, uint16_t length);
static void ota_fw_apply_handler(uint8_t *data, uint16_t length);

static void response_data_cmd(ota_cmd_t cmd, ota_err_t err)
{
    uint8_t data[2] = {cmd, err};
    svc_ota_data_notify(data, sizeof(data));
}

static void response_ctrl_error(ota_err_t err)
{
    uint8_t data[1] = {err};
    svc_ota_ctrl_notify(data, sizeof(data));
}

static void request_data(uint32_t offset, uint32_t size)
{
    uint8_t data[9], *pdata = data;

    packet_offset_request = offset;
    packet_size_request = size;

    put_u8_inc(&pdata, OTA_CMD_DATA);
    put_u32_inc(&pdata, offset);
    put_u32_inc(&pdata, size);
    svc_ota_data_notify(data, sizeof(data));
}

static void terminal_ota(void)
{
    ESP_LOGW(LOG_TAG, "ota terminal");
    xTimerStop(ota_timeout_timer, portMAX_DELAY);
    ota_stage = OTA_STAGE_IDLE;
    if (update_partition->type == ESP_PARTITION_TYPE_APP)
    {
        esp_ota_abort(update_handle);
    }
}

static void timeout_callback(TimerHandle_t xTimer)
{
    ESP_LOGW(LOG_TAG, "ota timeout");
    terminal_ota();
    response_data_cmd(OTA_CMD_DATA, OTA_ERR_TIMEOUT);
}

static void finish_callback(TimerHandle_t xTimer)
{
    ESP_LOGW(LOG_TAG, "ota reboot");
    esp_restart();
}

static void stage_idle_handler(uint8_t *data, uint16_t length)
{
    ESP_LOGI(LOG_TAG, "ready to ota");

    ota_cmd_t cmd = data[0];

    if (cmd != OTA_CMD_START)
    {
        ESP_LOGW(LOG_TAG, "error cmd %d", cmd);
        response_data_cmd(cmd, OTA_ERR_CMD);
        return;
    }
    if (length != 10)
    {
        ESP_LOGW(LOG_TAG, "error param length %d", length);
        response_data_cmd(cmd, OTA_ERR_PARAM);
        return;
    }

    uint8_t *pdata = data + 1;
    package_type = get_u8_inc(&pdata);
    package_size = get_u32_inc(&pdata);
    package_crc = get_u32_inc(&pdata);
    package_received = 0;
    ESP_LOGI(LOG_TAG, "package type %u, size %lu, crc 0x%08lx", package_type, package_size, package_crc);

    const esp_partition_t *partition;
    switch (package_type)
    {
        case OTA_TYPE_ESP:
        {
            partition = esp_ota_get_next_update_partition(NULL);
            break;
        }
        case OTA_TYPE_FW_APP:
        {
            partition = esp_partition_find_first(OTA_PARTITION_TYPE_FW, OTA_PARTITION_SUBTYPE_FW_APP, NULL);
            break;
        }
        default:
        {
            ESP_LOGW(LOG_TAG, "error package type %u", package_type);
            response_data_cmd(cmd, OTA_ERR_PARAM);
            return;
        }
    }
    if (partition == NULL)
    {
        ESP_LOGW(LOG_TAG, "can not find partition");
        response_data_cmd(cmd, OTA_ERR_UNKNOWN);
        return;
    }

    ESP_LOGI(LOG_TAG, "select partition %s", partition->label);
    if (package_size > partition->size)
    {
        ESP_LOGW(LOG_TAG, "error package size %lu > partition size %lu", package_size, partition->size);
        response_data_cmd(cmd, OTA_ERR_LENGTH);
        return;
    }

    if (partition->type == ESP_PARTITION_TYPE_APP)
    {
        if (esp_ota_begin(partition, package_size, &update_handle) != ESP_OK)
        {
            ESP_LOGW(LOG_TAG, "esp_ota_begin failed");
            response_data_cmd(cmd, OTA_ERR_UNKNOWN);
            return;
        }
    }
    else if (partition->type == OTA_PARTITION_TYPE_FW)
    {
        if (esp_partition_erase_range(partition, 0, partition->size) != ESP_OK)
        {
            ESP_LOGW(LOG_TAG, "esp_partition_erase_range failed");
            response_data_cmd(cmd, OTA_ERR_UNKNOWN);
            return;
        }

        uint8_t args[FW_HEADER_SIZE];
        uint8_t *pargs = args;
        put_u32_inc(&pargs, FW_MAGIC);
        put_u32_inc(&pargs, package_size);
        put_u32_inc(&pargs, package_crc);
        esp_partition_write(partition, 0, args, sizeof(args));
    }
    update_partition = partition;

    ESP_LOGI(LOG_TAG, "ready to receive package");
    xTimerStart(ota_timeout_timer, portMAX_DELAY);
    ota_stage = OTA_STAGE_DATA;
    response_data_cmd(OTA_CMD_START, OTA_OK);
    packet_received = 0;
    request_data(0, package_size < OTA_PACKET_SIZE ? package_size : OTA_PACKET_SIZE);
}

static void stage_data_handler(uint8_t *data, uint16_t length)
{
    if (packet_received == 0)
    {
        uint8_t *pdata = data;

        if (length < 11)
        {
            ESP_LOGW(LOG_TAG, "error param length %d", length);
            response_data_cmd(OTA_CMD_DATA, OTA_ERR_LENGTH);
            return;
        }

        ota_cmd_t cmd = get_u8_inc(&pdata);
        if (cmd == OTA_CMD_FINISH)
        {
            ESP_LOGW(LOG_TAG, "ota manual abort");
            terminal_ota();
            response_data_cmd(cmd, OTA_ERR_ABORT);
            return;
        }
        if (cmd != OTA_CMD_DATA)
        {
            ESP_LOGW(LOG_TAG, "error cmd %d", cmd);
            response_data_cmd(cmd, OTA_ERR_CMD);
            return;
        }

        uint32_t offset = get_u32_inc(&pdata);
        uint32_t size = get_u32_inc(&pdata);
        uint16_t crc = get_u16_inc(&pdata);
        ESP_LOGI(LOG_TAG, "packet response offset %lu, size %lu, crc 0x%04x", offset, size, crc);
        if (offset != packet_offset_request || size != packet_size_request)
        {
            ESP_LOGW(LOG_TAG, "error offset %lu != %lu or size %lu != %lu", offset, packet_offset_request, size, packet_size_request);
            response_data_cmd(OTA_CMD_DATA, OTA_ERR_PARAM);
            return;
        }
        packet_crc = crc;

        length -= (pdata - data);
        data = pdata;
    }

    if (length > packet_size_request - packet_received ||
        length > package_size - package_received)
    {
        ESP_LOGW(LOG_TAG, "error package/packet length %u, packet_received %lu, packet_size_request %lu, package_received %lu, package_size %lu",
                 length, packet_received, packet_size_request, package_received, package_size);
        terminal_ota();
        response_data_cmd(OTA_CMD_DATA, OTA_ERR_LENGTH);
        return;
    }

    memcpy(packet_data + packet_received, data, length);
    packet_received += length;

    // ESP_LOGI(LOG_TAG, "packet received %lu/%lu", packet_received, packet_size_request);

    if (packet_received == packet_size_request)
    {
        uint16_t crc = crc16((char *)packet_data, packet_received);
        if (crc != packet_crc)
        {
            packet_retry++;
            ESP_LOGW(LOG_TAG, "packet crc check error 0x%04x != 0x%04x, retry %lu times", crc, packet_crc, packet_retry);
            if (packet_retry >= 3)
            {
                ESP_LOGW(LOG_TAG, "packet retry too many times %lu", packet_retry);
                terminal_ota();
                response_data_cmd(OTA_CMD_DATA, OTA_ERR_CRC);
                return;
            }
        }
        else
        {
            packet_retry = 0;
            if (update_partition->type == ESP_PARTITION_TYPE_APP)
            {
                esp_ota_write(update_handle, packet_data, packet_received);
            }
            else if (update_partition->type == OTA_PARTITION_TYPE_FW)
            {
                esp_partition_write(update_partition, FW_HEADER_SIZE + packet_offset_request, packet_data, packet_received);
            }
            package_received += packet_received;
            ESP_LOGI(LOG_TAG, "package received %lu/%lu (%.2f)", package_received, package_size,
                     (float)package_received / (float)package_size * 100.0f);
        }

        xTimerStart(ota_timeout_timer, portMAX_DELAY);
        if (package_received < package_size)
        {
            packet_received = 0;
            request_data(package_received, package_size - package_received < OTA_PACKET_SIZE ?
                                           package_size - package_received : OTA_PACKET_SIZE);
        }
        else if (package_received == package_size)
        {
            ESP_LOGI(LOG_TAG, "package checking...");
            uint32_t csize = 0, ccrc = 0xffffffff;
            uint32_t offset = 0;
            if (update_partition->type == OTA_PARTITION_TYPE_FW)
            {
                offset = FW_HEADER_SIZE;
            }
            do
            {
                uint32_t rsize = package_size - csize < OTA_PACKET_SIZE ? package_size - csize : OTA_PACKET_SIZE;
                esp_partition_read(update_partition, offset + csize, packet_data, rsize);
                ccrc = crc32_append(ccrc, (char *)packet_data, rsize);
                csize += rsize;
            } while (csize < package_size);
            ccrc ^= 0xffffffff;
            if (ccrc != package_crc)
            {
                ESP_LOGW(LOG_TAG, "package crc check error 0x%08lx != 0x%08lx", ccrc, package_crc);
                terminal_ota();
                response_data_cmd(OTA_CMD_DATA, OTA_ERR_CRC);
                return;
            }
            ESP_LOGI(LOG_TAG, "package check pass");

            if (update_partition->type == ESP_PARTITION_TYPE_APP &&
                esp_ota_end(update_handle))
            {
                ESP_LOGW(LOG_TAG, "esp_ota_end failed");
                terminal_ota();
                response_data_cmd(OTA_CMD_DATA, OTA_ERR_UNKNOWN);
                return;
            }

            ESP_LOGI(LOG_TAG, "package receive finish");
            ota_stage = OTA_STAGE_FINISH;
            response_data_cmd(OTA_CMD_DATA, OTA_OK);
            xTimerStart(ota_timeout_timer, portMAX_DELAY);
        }
        else
        {
            ESP_LOGW(LOG_TAG, "error package_received %lu > package_size %lu", package_received, package_size);
            terminal_ota();
            response_data_cmd(OTA_CMD_DATA, OTA_ERR_LENGTH);
            return;
        }
    }
}

static void stage_finish_handler(uint8_t *data, uint16_t length)
{
    ESP_LOGI(LOG_TAG, "setup boot partition");

    ota_cmd_t cmd = get_u8(data);
    if (cmd != OTA_CMD_FINISH)
    {
        ESP_LOGW(LOG_TAG, "error cmd %d", cmd);
        response_data_cmd(OTA_CMD_FINISH, OTA_ERR_CMD);
        return;
    }
    if (update_partition->type == ESP_PARTITION_TYPE_APP &&
        esp_ota_set_boot_partition(update_partition) != ESP_OK)
    {
        ESP_LOGW(LOG_TAG, "esp_ota_set_boot_partition failed");
        response_data_cmd(OTA_CMD_FINISH, OTA_ERR_UNKNOWN);
        return;
    }

    ESP_LOGI(LOG_TAG, "ota success");
    xTimerStop(ota_timeout_timer, portMAX_DELAY);
    ota_stage = OTA_STAGE_IDLE;
    response_data_cmd(OTA_CMD_FINISH, OTA_OK);
}

static void ota_recv_handler(uint8_t *data, uint16_t length)
{
    ota_msg_t *msg = pvPortMalloc(sizeof(ota_msg_t) + length);
    CHECK_RET(msg);

    msg->func = ota_data_handler;
    msg->size = length;
    memcpy(msg->data, data, length);

    BaseType_t ret = xQueueSend(ota_msg_queue, &msg, 0);
    if (ret == errQUEUE_FULL)
    {
        ESP_LOGW(LOG_TAG, "ota msg queue full");
        vPortFree(msg);
    }
}

static void ota_ctrl_handler(uint8_t *data, uint16_t length)
{
    typedef enum
    {
        CTRL_CMD_APPLY_ESP = 0,
        CTRL_CMD_APPLY_FW,
    } ctrl_cmd_t;

    uint8_t *pdata = data;

    CHECK_RET(length >= 4); length -= 4;
    uint32_t magic = get_u32_inc(&pdata);
    if (magic != FW_MAGIC)
    {
        ESP_LOGW(LOG_TAG, "error magic 0x%08lx", magic);
        response_ctrl_error(OTA_ERR_CMD);
        return;
    }

    CHECK_RET(length >= 1); length -= 1;
    ctrl_cmd_t cmd = get_u8_inc(&pdata);
    switch (cmd)
    {
        case CTRL_CMD_APPLY_ESP:
        {
            CHECK_RET(length >= 4); length -= 4;
            uint32_t mask = get_u32_inc(&pdata);
            ESP_LOGI(LOG_TAG, "apply esp mask 0x%08lx", mask);

            if (mask & (1 << 0))
            {
                ESP_LOGI(LOG_TAG, "apply esp");
                response_ctrl_error(OTA_OK);
                xTimerStart(ota_finish_timer, portMAX_DELAY);
            }
            else
            {
                ESP_LOGW(LOG_TAG, "error mask 0x%08lx", mask);
                response_ctrl_error(OTA_ERR_CMD);
            }

            break;
        }
        case CTRL_CMD_APPLY_FW:
        {
            CHECK_RET(length >= 4); length -= 4;
            uint32_t mask = get_u32_inc(&pdata);
            ESP_LOGI(LOG_TAG, "apply fw mask 0x%08lx", mask);

            ota_msg_t *msg = pvPortMalloc(sizeof(ota_msg_t) + sizeof(mask));
            CHECK_RET(msg);
            msg->func = ota_fw_apply_handler;
            msg->size = sizeof(mask);
            put_u32(msg->data, mask);

            BaseType_t ret = xQueueSend(ota_msg_queue, &msg, 0);
            if (ret == errQUEUE_FULL)
            {
                ESP_LOGW(LOG_TAG, "ota msg queue full");
                response_ctrl_error(OTA_ERR_UNKNOWN);
                vPortFree(msg);
            }

            break;
        }
        default:
        {
            ESP_LOGW(LOG_TAG, "unknown cmd %u", cmd);
            response_ctrl_error(OTA_ERR_CMD);
            break;
        }
    }
}

static void ota_data_handler(uint8_t *data, uint16_t length)
{
    switch (ota_stage)
    {
        case OTA_STAGE_IDLE:
        {
            stage_idle_handler(data, length);
            break;
        }
        case OTA_STAGE_START:
        {
            break;
        }
        case OTA_STAGE_DATA:
        {
            stage_data_handler(data, length);
            break;
        }
        case OTA_STAGE_FINISH:
        {
            stage_finish_handler(data, length);
            break;
        }
    }
}

static void ota_fw_apply_handler(uint8_t *data, uint16_t length)
{
    bool result = false;
    uint32_t mask = get_u32(data);
    uint32_t tmask = 0;
    if (mask & (1 << OTA_TYPE_FW_APP))
        tmask |= (1 << DFU_TYPE_APP);
    ESP_LOGI(LOG_TAG, "verify dfu mask 0x%08lx", tmask);

    bool verify = dfu_verify(tmask);
    if (verify) {
        ESP_LOGI(LOG_TAG, "verify success");
    } else {
        ESP_LOGW(LOG_TAG, "verify failed");
        goto err0;
    }

    com_uart_lock_send();
    com_uart_pause_internal_recv(true);

    uint8_t blver[2];
    bool cmdok = false;
    for (uint16_t i = 0; i < 3 && !cmdok; i++)
    {
        dfu_enter_bootloader();
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (uint16_t i = 0; i < 3 && !cmdok; i++)
        {
            cmdok = dfu_get_bootloader_version(blver);
        }
    }
    if (cmdok) {
        ESP_LOGI(LOG_TAG, "enter boot success");
    } else {
        ESP_LOGW(LOG_TAG, "enter boot failed");
        goto err1;
    }

    ESP_LOGI(LOG_TAG, "start dfu mask@0x%08lx", tmask);
    cmdok = dfu_start(tmask);
    if (cmdok) {
        ESP_LOGI(LOG_TAG, "start dfu success");
        result = true;
    } else {
        ESP_LOGW(LOG_TAG, "start dfu failed");
    }

    dfu_exit_bootloader();
err1:
    com_uart_unlock_send();
    com_uart_pause_internal_recv(false);
err0:
    response_ctrl_error(result ? OTA_OK : OTA_ERR_UNKNOWN);
}

static void ota_task_handler(void *args)
{
    ota_msg_t *msg;
    for (;;)
    {
        xQueueReceive(ota_msg_queue, &msg, portMAX_DELAY);

        ota_msg_func_t func = msg->func;
        uint8_t *data = msg->data;
        uint32_t length = msg->size;

        func(data, length);

        vPortFree(msg);
    }
}

void prf_ota_init(void)
{
    ota_msg_queue = xQueueCreate(OTA_QUEUE_NUM, sizeof(ota_msg_t *));
    configASSERT(ota_msg_queue);

    ota_timeout_timer = xTimerCreate("ota_timeout", pdMS_TO_TICKS(OTA_TIMEOUT), pdFALSE, NULL, timeout_callback);
    configASSERT(ota_timeout_timer);

    ota_finish_timer = xTimerCreate("ota_finish", pdMS_TO_TICKS(1000), pdFALSE, NULL, finish_callback);
    configASSERT(ota_finish_timer);

    svc_ota_data_write_callback_register(ota_recv_handler);
    svc_ota_ctrl_write_callback_register(ota_ctrl_handler);

    xTaskCreate(ota_task_handler, "ota_task", 4096, NULL, 5, &ota_task_handle);
}

// void test_start_dfu(void)
// {
//     uint8_t data[] = {0x24, 0x10, 0x95, 0x19, 0x01, 0x0c, 0x00, 0x00, 0x00};

//     ota_ctrl_handler(data, sizeof(data));
// }
