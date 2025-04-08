#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "crc/crc32.h"
#include "com_uart.h"
#include "dfu.h"
#include "esp_log.h"

#define LOG_TAG "dfu"

// | head | opcode | length | param | crc |
// | 1    | 1      | 2      | n     | 4   |
#define DFU_PKT_HEADER_SIZE     1
#define DFU_PKT_OPCODE_SIZE     1
#define DFU_PKT_LENGTH_SIZE     2
#define DFU_PKT_CRC_SIZE        4
#define DFU_PKT_OVERHEAD_SIZE   (DFU_PKT_HEADER_SIZE + DFU_PKT_OPCODE_SIZE + DFU_PKT_LENGTH_SIZE + DFU_PKT_CRC_SIZE)

static uint8_t *setup_packet(dfu_opcode_t op, const uint8_t *param, uint16_t length, uint16_t *size)
{
    size_t pktsize = DFU_PKT_OVERHEAD_SIZE + length;
    uint8_t *pkt = malloc(pktsize);
    if (!pkt)
    {
        ESP_LOGI(LOG_TAG, "packet malloc failed");
        return NULL;
    }

    uint8_t *pdata = pkt;
    put_u8_inc(&pdata, 0xAA);
    put_u8_inc(&pdata, (uint8_t)op);
    put_u16_inc(&pdata, (uint16_t)length);
    if (param && length > 0)
    {
        memcpy(pdata, param, length);
        pdata += length;
    }
    put_u32_inc(&pdata, crc32((char *)pkt, pdata - pkt));

    *size = pktsize;
    return pkt;
}

static void free_packet(uint8_t *packet)
{
    free(packet);
}

static bool packet_parse(uint8_t packet[], uint16_t packet_length,
                         dfu_opcode_t *opcode, uint8_t **param, uint16_t *param_length)
{
    if (packet_length < DFU_PKT_OVERHEAD_SIZE)
    {
        ESP_LOGW(LOG_TAG, "packet length error %u", packet_length);
        return false;
    }

    if (packet[0] != 0xAA)
    {
        ESP_LOGW(LOG_TAG, "packet header error %02X", packet[0]);
        return false;
    }

    uint8_t *pdata = packet + 1;
    dfu_opcode_t op = get_u8_inc(&pdata);
    uint16_t plen = get_u16_inc(&pdata);
    if (plen + DFU_PKT_OVERHEAD_SIZE != packet_length)
    {
        ESP_LOGW(LOG_TAG, "packet length error %u %u", plen, packet_length);
        return false;
    }

    *param = pdata;
    pdata += plen;

    uint32_t ccrc = crc32((char *)packet, pdata - packet);
    uint32_t crc = get_u32_inc(&pdata);
    if (ccrc != crc)
    {
        ESP_LOGW(LOG_TAG, "packet crc error %08lX %08lX", crc, ccrc);
        return false;
    }

    *opcode = op;
    *param_length = plen;

    return true;
}

bool dfu_packet_request(dfu_opcode_t opcode, const uint8_t *input, uint16_t input_length,
                        uint8_t *output, uint16_t output_length, int timeout)
{
    bool result = false;

    uint16_t pktlen;
    uint8_t *pkt = setup_packet(opcode, input, input_length, &pktlen);
    CHECK_GO(pkt, err0);

    uint16_t buflen = DFU_PKT_OVERHEAD_SIZE + output_length;
    uint8_t *buf = malloc(buflen);
    CHECK_GO(buf, err1);

    com_uart_flush();
    com_uart_send_inlock(pkt, pktlen);
    com_uart_recv(buf, buflen, timeout);

    dfu_opcode_t op;
    uint8_t *param;
    uint16_t paramlen;
    result = packet_parse(buf, buflen, &op, &param, &paramlen);
    CHECK_GO(result, err2);
    CHECK_GO(op == opcode, err2);
    CHECK_GO(paramlen == output_length, err2);

    memcpy(output, param, paramlen);

err2:
    free(buf);
err1:
    free_packet(pkt);
err0:
    return result;
}
