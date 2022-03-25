/**
 * @file joybus_n64_accessory.h
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joybus N64 Accessory utilities
 * @ingroup joypad 
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_commands.h"
#include "joybus_n64_accessory.h"

uint16_t joybus_n64_accessory_addr_checksum(uint16_t addr)
{
    static const uint16_t xor_table[16] = { 
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x15, 0x1F, 0x0B,
        0x16, 0x19, 0x07, 0x0E,
        0x1C, 0x0D, 0x1A, 0x01
    };
    uint16_t checksum = 0;
    addr &= ~0x1F;
    for (int i = 15; i >= 5; i--)
        if ((addr >> i) & 0x1)
            checksum ^= xor_table[i];
    checksum &= 0x1F;
    return addr | checksum;
}

uint8_t joybus_n64_accessory_data_checksum(const uint8_t *data)
{
    uint8_t ret = 0;
    for (int i = 0; i <= 32; i++)
    {
        for (int j = 7; j >= 0; j--)
        {
            int tmp = 0;
            if (ret & 0x80) tmp = 0x85;
            ret <<= 1;
            if (i < 32 && data[i] & (0x01 << j)) ret |= 0x1;
            ret ^= tmp;
        }
    }
    return ret;
}

int joybus_n64_accessory_data_crc_compare(const uint8_t *data, uint8_t data_crc)
{
    uint8_t expected_crc = joybus_n64_accessory_data_checksum(data);
    if (expected_crc == data_crc) 
        return JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK;
    if (expected_crc == (data_crc ^ 0xFF))
        return JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_DISCONNECTED;
    return JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH;
}

void joybus_n64_accessory_read_async(int port, uint16_t addr, joybus_callback_t callback, void *ctx)
{
    assert(port >= 0 && port < 4);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_read_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_ID_N64_ACCESSORY_READ,
        .addr_checksum = joybus_n64_accessory_addr_checksum(addr),
    };
    // Micro-optimization: Minimize copy length
    const size_t recv_offset = offsetof(typeof(send_cmd), recv_bytes);
    memcpy(&input[i], &send_cmd, recv_offset);
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}

void joybus_n64_accessory_write_async(int port, uint16_t addr, uint8_t *data, joybus_callback_t callback, void *ctx)
{
    assert(port >= 0 && port < 4);
    uint8_t input[JOYBUS_BLOCK_SIZE] = {0};
    size_t i = port;

    const joybus_cmd_n64_accessory_write_port_t send_cmd = {
        .send_len = sizeof(send_cmd.send_bytes),
        .recv_len = sizeof(send_cmd.recv_bytes),
        .command = JOYBUS_COMMAND_ID_N64_ACCESSORY_WRITE,
        .addr_checksum = joybus_n64_accessory_addr_checksum(addr),
    };
    // Micro-optimization: Minimize copy length
    const size_t data_offset = offsetof(typeof(send_cmd), data);
    memcpy(&input[i], &send_cmd, data_offset);
    memcpy(&input[i + data_offset], data, sizeof(send_cmd.data));
    i += sizeof(send_cmd);

    // Close out the Joybus operation block
    input[i] = 0xFE;
    input[sizeof(input) - 1] = 0x01;

    joybus_exec_async(input, callback, ctx);
}
