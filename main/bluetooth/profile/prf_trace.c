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
#include "svc_trace.h"

#define LOG_TAG "TRACE"

static void shell_write_handler(uint8_t *data, uint16_t length)
{
    uint8_t *proto = com_proto_alloc(CHANNEL_BLE, length + 2);
    if (proto)
    {
        put_u16(proto, SVC_TRACE_CHAR_SHELL_UUID);
        memcpy(&proto[2], data, length);
        proto = com_proto_complete(proto);
        com_uart_send(proto, com_proto_length(proto));
        com_proto_free(proto);
    }
}

void prf_trace_init(void)
{
    svc_trace_shell_write_callback_register(shell_write_handler);
}

void prf_trace_shell_send(const char *cmd)
{
    shell_write_handler((uint8_t *)cmd, strlen(cmd));
}
