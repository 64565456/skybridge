#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "main.h"
#include "esp_log.h"
#include "prf_uuids.h"
#include "channel_defs.h"
#include "com_proto.h"
#include "com_uart.h"
#include "svc_ctrl.h"

#define LOG_TAG "CTRL"

static void name_write_handler(uint8_t *data, uint16_t length)
{
    ESP_LOG_BUFFER_HEX("name-rx", data, length);

    // 修改蓝牙广播名
    // 保存在nvs里面
}

static void find_write_handler(uint8_t *data, uint16_t length)
{
    ESP_LOG_BUFFER_HEX("find-rx", data, length);
    uint8_t *proto = com_proto_alloc(CHANNEL_BLE, length + 2);
    if (proto)
    {
        put_u16(proto, SVC_CTRL_CHAR_FIND_UUID);
        memcpy(&proto[2], data, length);
        proto = com_proto_complete(proto);
        com_uart_send(proto, com_proto_length(proto));
        com_proto_free(proto);
    }
}

static void misc_write_handler(uint8_t *data, uint16_t length)
{
    ESP_LOG_BUFFER_HEX("misc-rx", data, length);
    uint8_t *proto = com_proto_alloc(CHANNEL_BLE, length + 2);
    if (proto)
    {
        put_u16(proto, SVC_CTRL_CHAR_MISC_UUID);
        memcpy(&proto[2], data, length);
        proto = com_proto_complete(proto);
        com_uart_send_inlock(proto, com_proto_length(proto));
        com_proto_free(proto);
    }
}

void prf_ctrl_init(void)
{
    svc_ctrl_name_write_callback_register(name_write_handler);
    svc_ctrl_find_write_callback_register(find_write_handler);
    svc_ctrl_misc_write_callback_register(misc_write_handler);
}

void prf_ctrl_misc_set_reboot(bool trap_boot)
{
    misc_write_handler((uint8_t[]){ 0x01, trap_boot }, 2);
}
