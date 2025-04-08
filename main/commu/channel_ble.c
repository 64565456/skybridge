#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "main.h"
#include "svc_ctrl.h"
#include "svc_trace.h"
#include "svc_ota.h"
#include "prf_uuids.h"

#define LOG_TAG "CH-BLE"

void channel_ble_handler(uint8_t *data, uint16_t length)
{
    CHECK_RET(length >= 2);

    uint16_t uuid = get_u16(data);
    data += 2; length -= 2;

    switch (uuid)
    {
        case SVC_CTRL_CHAR_NAME_UUID:
        {
            ESP_LOG_BUFFER_HEX("name-tx", data, length);
            svc_ctrl_name_notify(data, length);
            break;
        }
        case SVC_CTRL_CHAR_FIND_UUID:
        {
            ESP_LOG_BUFFER_HEX("find-tx", data, length);
            svc_ctrl_find_notify(data, length);
            break;
        }
        case SVC_TRACE_CHAR_LOG_UUID:
        {
            svc_trace_log_notify(data, length);
            break;
        }
        case SVC_TRACE_CHAR_SHELL_UUID:
        {
            svc_trace_shell_notify(data, length);
            break;
        }
        default:
        {
            ESP_LOGE(LOG_TAG, "unknown uuid 0x%04x", uuid);
            break;
        }
    }
}
