#ifndef __CHANNEL_DEFS_H
#define __CHANNEL_DEFS_H


typedef enum
{
    CHANNEL_ESP,
    CHANNEL_BLE,
    CHANNEL_BT,
    CHANNEL_WIFI,
    CHANNEL_ETH,
    CHANNEL_NUM,

    CHANNEL_ACK = 0xff,
} channel_type_t;


#endif /* __CHANNEL_DEFS_H */
