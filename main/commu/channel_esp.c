#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ble_main.h"
#include "channel_defs.h"
#include "com_uart.h"
#include "com_proto.h"

#define LOG_TAG "CH-ESP"

typedef enum
{
    ESP_STATUS = 0x00,
    ESP_POWEROFF = 0x01,
    ESP_REBOOT = 0x02,
    ESP_SLEEP = 0x03,
    ESP_SET_NVS = 0x10,              /* uint16: key_len uint8[]: key uint16: val_len uint8[]: val */
    ESP_GET_NVS = 0x11,
    ESP_BLE_GET_CONNECT = 0x40,      /* bool: connected */
    ESP_BLE_GET_MTU = 0x41,          /* uint16: mtu */
    ESP_BLE_SET_DEV_NAME = 0x50,     /* string: name */
    ESP_BLE_SET_USER_DATA = 0x51,    /* bytes: data */
    ESP_BLE_SET_PPCP = 0x52,         /* uint16: min, max, laytency, timeout */
} esp_cmd_t;

typedef struct
{
    uint8_t cmd;
    uint8_t data[];
} __attribute__((packed)) esp_ack_t;

#define DEV_NAME_LEN_MAX    20
#define USER_DATA_LEN_MAX   27
static char dev_name[DEV_NAME_LEN_MAX + 1] = "ELITE_UNKNOWN";
static uint8_t user_data[USER_DATA_LEN_MAX + 1];

static void esp_ack(esp_cmd_t cmd, uint8_t *data, uint16_t length)
{
    esp_ack_t *ack = com_proto_alloc(CHANNEL_ESP, sizeof(esp_ack_t) + length);
    CHECK_RET(ack);

    ack->cmd = cmd;
    memcpy(ack->data, data, length);

    uint8_t *proto = com_proto_complete(ack);
    com_uart_send(proto, com_proto_length(proto));
    com_proto_free(proto);
}

static void esp_ack_bool(esp_cmd_t cmd, bool status)
{
    uint8_t data[1] = { status ? 1 : 0 };
    esp_ack(cmd, data, sizeof(data));
}

void channel_esp_handler(uint8_t *data, uint16_t length)
{
    esp_cmd_t cmd = data[0];
    data++; length--;

    switch (cmd)
    {
        case ESP_STATUS:
        {
            esp_ack_bool(ESP_STATUS, true);
            break;
        }
        case ESP_REBOOT:
        {
            esp_restart();
            esp_ack_bool(ESP_REBOOT, true);
            break;
        }
        case ESP_SLEEP:
        {
            break;
        }
        case ESP_SET_NVS:
        {
            break;
        }
        case ESP_GET_NVS:
        {
            break;
        }
        case ESP_BLE_GET_CONNECT:
        {
            esp_ack_bool(ESP_BLE_GET_CONNECT, ble_is_connect());
            break;
        }
        case ESP_BLE_GET_MTU:
        {
            uint16_t mtu = ble_get_mtu();
            esp_ack(ESP_BLE_GET_MTU, (uint8_t *)&mtu, sizeof(mtu));
            break;
        }
        case ESP_BLE_SET_DEV_NAME:
        {
            uint16_t len = length < DEV_NAME_LEN_MAX ? length : DEV_NAME_LEN_MAX;
            memcpy(dev_name, data, len);
            dev_name[len] = 0;
            ble_set_adv_data(dev_name, user_data, USER_DATA_LEN_MAX);
            esp_ack_bool(ESP_BLE_SET_DEV_NAME, true);
            break;
        }
        case ESP_BLE_SET_USER_DATA:
        {
            memcpy(user_data, data, length);
            ble_set_adv_data(dev_name, user_data, USER_DATA_LEN_MAX);
            esp_ack_bool(ESP_BLE_SET_USER_DATA, true);
            break;
        }
        case ESP_BLE_SET_PPCP:
        {
            if (length == 8)
            {
                uint8_t *pdata = data;
                uint16_t interval_min = get_u16_inc(&pdata);
                uint16_t interval_max = get_u16_inc(&pdata);
                uint16_t latency = get_u16_inc(&pdata);
                uint16_t timeout = get_u16_inc(&pdata);
                ble_set_ppcp(interval_min, interval_max, latency, timeout);
                esp_ack_bool(ESP_BLE_SET_PPCP, true);
            }
            else
            {
                esp_ack_bool(ESP_BLE_SET_PPCP, false);
            }
            break;
        }

        default:
            break;
    }
}
