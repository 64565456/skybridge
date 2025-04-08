#ifndef __COM_PROTO_H
#define __COM_PROTO_H


#include <stdint.h>

/* 自定义的OTA升级协议的格式  
| magic | channel | length | data     | crc16 |
| 1     | 1       | 2      | {length} | 2     |
| 0xaa  | 0~255   | 0~65535|          |       |

        （相比较STM32升级协议的数据格式：opcode改成了channel） 
channel：转发通道：主控STM32发送给ESP32的数据，ESP32通过什么方式（蓝牙、WIFI、网卡等）转发出去
*/

#define COM_PROTO_MAGIC             0xaa
#define COM_PROTO_MAGIC_OFFSET      0
#define COM_PROTO_MAGIC_SIZE        1
#define COM_PROTO_CHANNEL_OFFSET    1
#define COM_PROTO_CHANNEL_SIZE      1
#define COM_PROTO_LENGTH_OFFSET     2
#define COM_PROTO_LENGTH_SIZE       2
#define COM_PROTO_CRC_OFFSET        4
#define COM_PROTO_CRC_SIZE          2
#define COM_PROTO_DATA_OFFSET       4
#define COM_PROTO_BASE_SIZE         (COM_PROTO_MAGIC_SIZE + COM_PROTO_CHANNEL_SIZE + COM_PROTO_LENGTH_SIZE + COM_PROTO_CRC_SIZE)
#define COM_PROTO_HEAD_SIZE         (COM_PROTO_MAGIC_SIZE + COM_PROTO_CHANNEL_SIZE + COM_PROTO_LENGTH_SIZE)

void *com_proto_alloc(uint8_t channel, size_t length);
uint8_t *com_proto_complete(void *data);
uint16_t com_proto_length(uint8_t *proto);
void com_proto_free(uint8_t *proto);
bool com_proto_verify(uint8_t *proto, uint16_t length);


#endif /* __COM_PROTO_H */
