#ifndef __DFU_H
#define __DFU_H


typedef enum
{
    dfu_opcode_none = 0x00,
    dfu_opcode_inquiry = 0x10,
    dfu_opcode_boot = 0x11,
    dfu_opcode_unlock = 0x1a,
    dfu_opcode_reset = 0x1f,
    dfu_opcode_erase = 0x20,
    dfu_opcode_read,
    dfu_opcode_write,
    dfu_opcode_verify,
    dfu_opcode_end,
} dfu_opcode_t;

typedef enum
{
    dfu_inquiry_version,
    dfu_inquiry_mtu_size,
} dfu_inquiry_t;

typedef enum
{
    DFU_TYPE_NONE,
    DFU_TYPE_APP,
} dfu_type_t;

bool dfu_packet_request(dfu_opcode_t opcode, const uint8_t *input, uint16_t input_length,
                        uint8_t *output, uint16_t output_length, int timeout);

bool dfu_get_bootloader_version(uint8_t version[2]);
bool dfu_get_mtu_size(uint16_t *mtu_size);
bool dfu_reboot_system(void);

bool dfu_erase_flash(uint32_t address, uint32_t size);
bool dfu_write_flash(uint32_t address, uint8_t *data, uint16_t length);
bool dfu_verify_flash(uint32_t address, uint32_t size, uint32_t crc);

void dfu_enter_bootloader(void);
void dfu_exit_bootloader(void);
bool dfu_verify(uint32_t mask);
bool dfu_start(uint32_t mask);


#endif /* __DFU_H */
