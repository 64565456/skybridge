/*
 * serial protocol
 *
 * | head | channel | length | data | crc16 |
 * | 0xaa | esp/ble/bt/wifi | 2byte | n byte | 2byte |
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "esp_log.h"
#include "channel_forward.h"
#include "crc/crc16.h"
#include "com_proto.h"

#define LOG_TAG "PROTO"

void *com_proto_alloc(uint8_t channel, size_t length)
{
    uint8_t *proto = malloc(length + COM_PROTO_BASE_SIZE);
    CHECK_RETX(proto, NULL);

    uint8_t *pproto = proto;
    put_u8_inc(&pproto, COM_PROTO_MAGIC);
    put_u8_inc(&pproto, channel);
    put_u16_inc(&pproto, length);

    return pproto;
}

uint8_t *com_proto_complete(void *data)
{
    CHECK_RETX(data, NULL);

    uint8_t *proto = data - COM_PROTO_HEAD_SIZE;
    CHECK_RETX(proto[COM_PROTO_MAGIC_OFFSET] == COM_PROTO_MAGIC, NULL);

    uint16_t length = get_u16(&proto[COM_PROTO_LENGTH_OFFSET]);

    uint16_t crc = crc16((char *)&proto[COM_PROTO_CHANNEL_OFFSET], length + COM_PROTO_CHANNEL_SIZE + COM_PROTO_LENGTH_SIZE);
    put_u16(&proto[length + COM_PROTO_CRC_OFFSET], crc);

    return proto;
}

uint16_t com_proto_length(uint8_t *proto)
{
    CHECK_RETX(proto, 0);
    CHECK_RETX(proto[COM_PROTO_MAGIC_OFFSET] == COM_PROTO_MAGIC, 0);

    return get_u16(&proto[COM_PROTO_LENGTH_OFFSET]) + COM_PROTO_BASE_SIZE;
}

void com_proto_free(uint8_t *proto)
{
    free(proto);
}

bool com_proto_verify(uint8_t *proto, uint16_t length)
{
    CHECK_RETX(proto, false);
    CHECK_RETX(length >= COM_PROTO_BASE_SIZE, false);
    CHECK_RETX(proto[COM_PROTO_MAGIC_OFFSET] == COM_PROTO_MAGIC, false);
    CHECK_RETX(get_u16(&proto[COM_PROTO_LENGTH_OFFSET]) == length - COM_PROTO_BASE_SIZE, false); 

    //接收CRC和计算CRC 进行比较
    uint16_t crc = crc16((char *)&proto[COM_PROTO_CHANNEL_OFFSET], length - COM_PROTO_MAGIC_SIZE - COM_PROTO_CRC_SIZE);//计算CRC
    uint16_t crc2 = get_u16(&proto[length - COM_PROTO_CRC_SIZE]);//取出CRC数据
    return crc == crc2;
}
