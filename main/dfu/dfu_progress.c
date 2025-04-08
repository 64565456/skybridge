#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "dfu.h"


bool dfu_get_bootloader_version(uint8_t version[2])
{
    uint8_t input[1] = { (uint8_t)dfu_inquiry_version };
    uint8_t output[2];

    bool result = dfu_packet_request(dfu_opcode_inquiry, input, sizeof(input), output, sizeof(output), 1000);
    CHECK_RETX(result, false);

    memcpy(version, output, sizeof(output));

    return true;
}

bool dfu_get_mtu_size(uint16_t *mtu_size)
{
    uint8_t input[1] = { (uint8_t)dfu_inquiry_mtu_size };
    uint8_t output[2];

    bool result = dfu_packet_request(dfu_opcode_inquiry, input, sizeof(input), output, sizeof(output), 1000);
    CHECK_RETX(result, false);

    *mtu_size = get_u16(output);

    return true;
}

bool dfu_reboot_system(void)
{
    bool result = dfu_packet_request(dfu_opcode_reset, NULL, 0, NULL, 0, 1000);
    CHECK_RETX(result, false);

    return true;
}

bool dfu_erase_flash(uint32_t address, uint32_t size)
{
    uint8_t input[8];
    uint8_t output[1];

    put_u32(input, address);
    put_u32(input + 4, size);

    bool result = dfu_packet_request(dfu_opcode_erase, input, sizeof(input), output, sizeof(output), 8 * 1000);
    CHECK_RETX(result, false);
    CHECK_RETX(output[0] == 0, false);

    return true;
}

bool dfu_write_flash(uint32_t address, uint8_t *data, uint16_t length)
{
    uint8_t *input = malloc(length + 8);
    uint8_t output[1];
    CHECK_RETX(input, false);

    put_u32(input, address);
    put_u32(input + 4, length);
    memcpy(input + 8, data, length);

    bool result = dfu_packet_request(dfu_opcode_write, input, length + 8, output, sizeof(output), 1000);
    free(input);
    CHECK_RETX(result, false);

    return output[0] == 0;
}

bool dfu_verify_flash(uint32_t address, uint32_t size, uint32_t crc)
{
    uint8_t input[12];
    uint8_t output[1];

    put_u32(input, address);
    put_u32(input + 4, size);
    put_u32(input + 8, crc);

    bool result = dfu_packet_request(dfu_opcode_verify, input, sizeof(input), output, sizeof(output), 3 * 1000);
    CHECK_RETX(result, false);

    return output[0] == 0;
}
