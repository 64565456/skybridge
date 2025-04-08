#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "channel_defs.h"

#define LOG_TAG "CH-FORD"

extern void channel_esp_handler(uint8_t *data, uint16_t length);
extern void channel_ble_handler(uint8_t *data, uint16_t length);

void channel_forward(uint8_t channel, uint8_t *data, uint16_t length)
{
    switch (channel)
    {
        case CHANNEL_ESP:
        {
            channel_esp_handler(data, length);
            break;
        }
        case CHANNEL_BLE:
        {
            channel_ble_handler(data, length);
            break;
        }
        default:
            break;
    }
}
