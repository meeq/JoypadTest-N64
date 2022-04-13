
/**
 * @file joypad_accessory.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Accessory helpers
 * @ingroup joypad
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_n64_accessory.h"
#include "joypad_accessory.h"
#include "joypad_internal.h"

static void joypad_accessory_detect_read_callback(uint64_t *out_dwords, void *ctx);
static void joypad_accessory_detect_write_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_enable_read_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_enable_write_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_wait_timer_callback(int ovfl, void *ctx);
static void joypad_n64_transfer_pak_load_read_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_load_write_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_store_read_callback(uint64_t *out_dwords, void *ctx);
static void joypad_n64_transfer_pak_store_write_callback(uint64_t *out_dwords, void *ctx);

static bool joypad_accessory_read_crc_error_check(
    joypad_port_t port,
    const joybus_cmd_n64_accessory_read_port_t *recv_cmd,
    joybus_callback_t retry_callback,
    void *retry_ctx
)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    switch (joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc))
    {
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK:
        {
            // Accessory is no longer connected!
            device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
            device->rumble_active = false;
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_NONE;
            accessory->status = JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_ABSENT;
            accessory->error = JOYPAD_ACCESSORY_ERROR_ABSENT;
            return true;
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH:
        {
            size_t retries = accessory->retries;
            if (retries < JOYPAD_ACCESSORY_RETRY_LIMIT)
            {
                // Retry: Bad communication with the accessory
                accessory->retries = retries + 1;
                accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
                uint16_t retry_addr = recv_cmd->addr_checksum;
                retry_addr &= JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET;
                joybus_n64_accessory_read_async(
                    port, retry_addr,
                    retry_callback, retry_ctx
                );
                return true;
            }
            else
            {
                // Retry limit exceeded; read failed
                accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
                accessory->error = JOYPAD_ACCESSORY_ERROR_CHECKSUM;
                return true;
            }
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK:
        default:
        {
            accessory->error = JOYPAD_ACCESSORY_ERROR_NONE;
            return false;
        }
    }
}

static bool joypad_accessory_write_crc_error_check(
    joypad_port_t port,
    const joybus_cmd_n64_accessory_write_port_t *recv_cmd,
    joybus_callback_t retry_callback,
    void *retry_ctx
)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    switch (joybus_n64_accessory_data_crc_compare(recv_cmd->data, recv_cmd->data_crc))
    {
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK:
        {
            // Accessory is no longer connected!
            device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
            device->rumble_active = false;
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_NONE;
            accessory->status = JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_ABSENT;
            accessory->error = JOYPAD_ACCESSORY_ERROR_ABSENT;
            return true;
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH:
        {
            size_t retries = accessory->retries;
            if (retries < JOYPAD_ACCESSORY_RETRY_LIMIT)
            {
                // Retry: Bad communication with the accessory
                accessory->retries = retries + 1;
                accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
                uint16_t retry_addr = recv_cmd->addr_checksum;
                retry_addr &= JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET;
                joybus_n64_accessory_write_async(
                    port, retry_addr, recv_cmd->data,
                    retry_callback, retry_ctx
                );
                return true;
            }
            else
            {
                // Retry limit exceeded; write failed
                accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
                accessory->error = JOYPAD_ACCESSORY_ERROR_CHECKSUM;
                return true;
            }
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK:
        default:
        {
            accessory->error = JOYPAD_ACCESSORY_ERROR_NONE;
            return false;
        }
    }
}


void joypad_n64_transfer_pak_wait_timer_init(joypad_port_t port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    // Ensure there is a disabled timer ready to restart:
    if (!accessory->transfer_pak_wait_timer)
    {
        // The Transfer Pak takes about 200 milliseconds to fully power-on
        // after being probed; sadly, we must use a hard-coded delay
        accessory->transfer_pak_wait_timer = new_timer_context(
            TIMER_TICKS(200 * 1000), TF_ONE_SHOT | TF_DISABLED,
            joypad_n64_transfer_pak_wait_timer_callback, (void *)port
        );
    }
}

static void joypad_n64_transfer_pak_wait_timer_callback(int ovfl, void *ctx)
{
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_WAIT)
    {
        // Step 4D: Write access flag to Transfer Pak status register
        uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(write_data, JOYBUS_N64_TRANSFER_PAK_STATUS_ACCESS, sizeof(write_data));
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS, write_data,
            joypad_accessory_detect_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_WAIT)
    {
        // Step 4F: Read Transfer Pak status register to see if there is a cartridge
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WAIT)
    {
        uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(write_data, JOYBUS_N64_TRANSFER_PAK_STATUS_ACCESS, sizeof(write_data));
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS, write_data,
            joypad_n64_transfer_pak_enable_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WAIT)
    {
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS,
            joypad_n64_transfer_pak_enable_read_callback, ctx
        );
    }
}

static void joypad_accessory_detect_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_detecting(state)) return;

    uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_enable_read_callback;
    if (joypad_accessory_read_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ)
    {
        // Compare the expected label with what was actually read back
        for (size_t i = 0; i < sizeof(write_data); ++i) write_data[i] = i;
        if (memcmp(recv_cmd->data, write_data, sizeof(write_data)) == 0)
        {
            // Success: Label write persisted; this appears to be a Controller Pak
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_CONTROLLER_PAK;
        }
        else
        {
            // Step 3A: Write probe value to detect Rumble Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_PROBE_WRITE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_PROBE_READ)
    {
        if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK)
        {
            // Success: Probe reports that this is a Rumble Pak
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_RUMBLE_PAK;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK;
        }
        else if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR)
        {
            // Success: Bio Sensor responds to all reads with probe value
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_BIO_SENSOR;
        }
        else if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_SNAP_STATION)
        {
            // Success: Snap Station responds to all probes with probe value
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_BIO_SENSOR;
        }
        else
        {
            // Step 4A: Write probe value to detect Transfer Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_ON;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_READ)
    {
        if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON)
        {
            // Step 4C: Probe reports that this is a Transfer Pak; wait to access cart!
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_WAIT;
            timer_link_t *timer = accessory->transfer_pak_wait_timer;
            assertf(timer, "transfer_pak_wait_timer is NULL on port %d", port + 1);
            restart_timer(timer);
        }
        else
        {
            // Failure: Unable to determine which accessory is connected
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->type = JOYPAD_ACCESSORY_TYPE_UNKNOWN;
            accessory->transfer_pak_status.raw = JOYBUS_N64_TRANSFER_PAK_STATUS_OFF;
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_READ)
    {
        joybus_n64_transfer_pak_status_t status = { .raw = recv_cmd->data[0] };
        accessory->transfer_pak_status = status;
        if (status.power && status.access)
        {
            // Step 4G: Write bank 0 to Transfer Pak bank register
            // The goal here is to read the cartridge header
            uint8_t bank = 0;
            uint16_t cart_addr = 0x0100;
            uint16_t tpak_addr = cart_addr + JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART;
            uint8_t *dst = (void *)&accessory->transfer_pak_cart;
            size_t len = sizeof(accessory->transfer_pak_cart);
            accessory->transfer_pak_io = (joypad_n64_transfer_pak_io_t){
                .start = dst,
                .end = dst + len,
                .cursor = dst,
                .bank = bank,
                .cart_addr = cart_addr,
                .tpak_addr = tpak_addr,
            };

            memset(write_data, bank, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_BANK_WRITE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
        else
        {
            // Skip to Step 4I: Write probe value to turn off Transfer Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_OFF;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, (void *)port
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_DATA_READ)
    {
        volatile joypad_n64_transfer_pak_io_t *io = &accessory->transfer_pak_io;
        memcpy(io->cursor, recv_cmd->data, JOYBUS_N64_ACCESSORY_DATA_SIZE);
        uint8_t *cursor = io->cursor += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        uint16_t tpak_addr = io->tpak_addr += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        if (cursor < io->end)
        {
            // Continue reading cartridge header data
            accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_DATA_READ;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_read_async(
                port, tpak_addr,
                joypad_accessory_detect_read_callback, ctx
            );
        }
        else
        {
            // Finished reading cartridge header data
            gb_cart_type_t cart_type = accessory->transfer_pak_cart.cartridge_type;
            if (
                cart_type == GB_MBC5_RUMBLE ||
                cart_type == GB_MBC5_RUMBLE_RAM ||
                cart_type == GB_MBC5_RUMBLE_RAM_BATTERY
            )
            {
                // If the cartridge supports rumble, leave the Transfer Pak on
                accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
                accessory->type = JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK;
                device->rumble_method = JOYPAD_RUMBLE_METHOD_N64_TRANSFER_PAK_MBC5;
            }
            else
            {
                // Step 4I: Write probe value to turn off Transfer Pak
                memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF, sizeof(write_data));
                accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_OFF;
                accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
                accessory->retries = 0;
                joybus_n64_accessory_write_async(
                    port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                    joypad_accessory_detect_write_callback, (void *)port
                );
            }
        }
    }
}

static void joypad_accessory_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_detecting(state)) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_enable_write_callback;
    if (joypad_accessory_write_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_INIT)
    {
        // Step 2A: Overwrite "label" area to detect Controller Pak
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        for (size_t i = 0; i < sizeof(data); ++i) data[i] = i;
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_LABEL, data,
            joypad_accessory_detect_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE)
    {
        // Step 2B: Read back the "label" area to detect Controller Pak
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_LABEL,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_PROBE_WRITE)
    {
        // Step 3B: Read probe value to detect Rumble Pak
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_PROBE_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_ON)
    {
        // Step 4B: Read probe value to detect Transfer Pak
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_WRITE)
    {
        // Step 4E: Wait for Game Boy cartridge in Transfer Pak to reset
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_STATUS_WAIT;
        timer_link_t *timer = accessory->transfer_pak_wait_timer;
        assertf(timer, "transfer_pak_wait_timer is NULL on port %d", port + 1);
        restart_timer(timer);
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_BANK_WRITE)
    {
        // Step 4H: Read cartridge header through Transfer Pak data register
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_DATA_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, accessory->transfer_pak_io.tpak_addr,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_PROBE_OFF)
    {
        // Success: Transfer Pak has been probed and powered off
        accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
        accessory->type = JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK;
        accessory->transfer_pak_status.raw = JOYBUS_N64_TRANSFER_PAK_STATUS_OFF;
    }
}

/**
 * @brief Detect which accessory is inserted in an N64 controller.
 * 
 * Step 1: Ensure Transfer Pak is turned off
 * Step 2A: Overwrite "label" area to detect Controller Pak
 * Step 2B: Read back the "label" area to detect Controller Pak
 * Step 3A: Write probe value to detect Rumble Pak
 * Step 3B: Read probe value to detect Rumble Pak
 * Step 4A: Write probe value to detect Transfer Pak
 * Step 4B: Read probe value to detect Transfer Pak
 * Step 4C: Wait for Transfer Pak to finish powering on
 * Step 4D: Write access flag to Transfer Pak status register
 * Step 4E: Wait for Game Boy cartridge in Transfer Pak to reset
 * Step 4F: Read Transfer Pak status register to see if there is a cartridge
 * Step 4G: Write bank 0 to Transfer Pak bank register
 * Step 4H: Read cartridge header through Transfer Pak data register
 * Step 4I: Write probe value to turn off Transfer Pak
 * 
 * @param[in] port Which controller port to detect the accessory on
 */
void joypad_accessory_detect_async(joypad_port_t port)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    if (accessory->state == JOYPAD_ACCESSORY_STATE_IDLE)
    {
        joypad_n64_transfer_pak_wait_timer_init(port);
        // Step 1: Ensure Transfer Pak is turned off
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF, sizeof(data));
        accessory->state = JOYPAD_ACCESSORY_STATE_DETECT_INIT;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, data,
            joypad_accessory_detect_write_callback, (void *)port
        );
    }
}

static void joypad_n64_rumble_pak_toggle_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (state != JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_enable_write_callback;
    if (joypad_accessory_write_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else
    {
        accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
    }
}

void joypad_n64_rumble_pak_toggle_async(joypad_port_t port, bool active)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    device->rumble_active = active;
    accessory->state = JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE;
    accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
    accessory->retries = 0;
    uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(motor_data, active, sizeof(motor_data));
    joybus_n64_accessory_write_async(
        port, JOYBUS_N64_ACCESSORY_ADDR_RUMBLE, motor_data,
        joypad_n64_rumble_pak_toggle_write_callback, (void *)port
    );
}

static void joypad_n64_transfer_pak_enable_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_enabling(state)) return;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_enable_read_callback;
    if (joypad_accessory_read_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_READ)
    {
        accessory->transfer_pak_status.raw = recv_cmd->data[0];
        accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_n64_transfer_pak_enable_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_enabling(state)) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_enable_write_callback;
    if (joypad_accessory_write_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WRITE)
    {
        if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON)
        {
            accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WAIT;
            timer_link_t *timer = accessory->transfer_pak_wait_timer;
            assertf(timer, "transfer_pak_wait_timer is NULL on port %d", port + 1);
            restart_timer(timer);
        }
        else
        {
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->transfer_pak_status.raw = JOYBUS_N64_TRANSFER_PAK_STATUS_OFF;
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WRITE)
    {
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_STATUS_WAIT;
        timer_link_t *timer = accessory->transfer_pak_wait_timer;
        assertf(timer, "transfer_pak_wait_timer is NULL on port %d", port + 1);
        restart_timer(timer);
    }
}

void joypad_n64_transfer_pak_enable_async(joypad_port_t port, bool enabled)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    
    // Turn the Transfer Pak on or off with magic probe values
    uint8_t probe_value = enabled
        ? JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON
        : JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF
        ;
    uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(write_data, probe_value, sizeof(write_data));

    accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_ENABLE_PROBE_WRITE;
    accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
    accessory->retries = 0;
    joybus_n64_accessory_write_async(
        port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
        joypad_n64_transfer_pak_enable_write_callback, (void *)port
    );
}

static void joypad_n64_transfer_pak_load_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_loading(state)) return;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_load_read_callback;
    if (joypad_accessory_read_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    volatile joypad_n64_transfer_pak_io_t *io = &accessory->transfer_pak_io;
    if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_STATUS_READ)
    {
        joybus_n64_transfer_pak_status_t status = { .raw = recv_cmd->data[0] };
        accessory->transfer_pak_status = status;
        if (!status.access || !status.power || status.cart_pulled)
        {
            // The Game Boy cartridge is no longer accessible; bail!
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_TRANSFER_PAK_STATUS_CHANGE;
            return;
        }

        // Proceed with reading; select a Transfer Pak data bank
        uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(write_data, io->bank, sizeof(write_data));
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_BANK_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK, write_data,
            joypad_n64_transfer_pak_load_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_DATA_READ)
    {
        memcpy(io->cursor, recv_cmd->data, JOYBUS_N64_ACCESSORY_DATA_SIZE);
        uint8_t *cursor = io->cursor += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        uint16_t tpak_addr = io->tpak_addr += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        uint16_t cart_addr = io->cart_addr += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        int next_bank = cart_addr / JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
        if (cursor >= io->end)
        {
            // Finished reading data
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
        }
        else if (next_bank == io->bank)
        {
            // Continue reading data
            accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_DATA_READ;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_read_async(
                port, tpak_addr,
                joypad_n64_transfer_pak_load_read_callback, ctx
            );
        }
        else
        {
            // Switch to the next bank
            io->tpak_addr = JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART;
            io->bank = next_bank;
            uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
            memset(write_data, next_bank, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_BANK_WRITE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK, write_data,
                joypad_n64_transfer_pak_load_write_callback, ctx
            );
        }
    }
}

static void joypad_n64_transfer_pak_load_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_loading(state)) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_load_write_callback;
    if (joypad_accessory_write_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_BANK_WRITE)
    {
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_DATA_READ;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_read_async(
            port, accessory->transfer_pak_io.tpak_addr,
            joypad_n64_transfer_pak_load_read_callback, ctx
        );
    }
}

void joypad_n64_transfer_pak_load_async(joypad_port_t port, uint16_t cart_addr, void *dst, size_t len)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    assert(cart_addr % JOYBUS_N64_ACCESSORY_DATA_SIZE == 0);
    assert(len % JOYBUS_N64_ACCESSORY_DATA_SIZE == 0);
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];

    uint8_t bank = cart_addr / JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
    uint16_t tpak_addr = JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART;
    tpak_addr += cart_addr % JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
    accessory->transfer_pak_io = (joypad_n64_transfer_pak_io_t){
        .start = dst,
        .end = dst + len,
        .cursor = dst,
        .bank = bank,
        .cart_addr = cart_addr,
        .tpak_addr = tpak_addr,
    };

    accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_LOAD_STATUS_READ;
    accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
    accessory->retries = 0;
    joybus_n64_accessory_read_async(
        port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS,
        joypad_n64_transfer_pak_load_read_callback, (void *)port
    );
}

static void joypad_n64_transfer_pak_store_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_storing(state)) return;

    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_store_read_callback;
    if (joypad_accessory_read_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_STATUS_READ)
    {
        joybus_n64_transfer_pak_status_t status = { .raw = recv_cmd->data[0] };
        accessory->transfer_pak_status = status;
        if (!status.access || !status.power || status.cart_pulled)
        {
            // The Game Boy cartridge is no longer accessible; bail!
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_TRANSFER_PAK_STATUS_CHANGE;
            return;
        }

        // Proceed with writing; select a Transfer Pak data bank
        volatile joypad_n64_transfer_pak_io_t *io = &accessory->transfer_pak_io;
        uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(write_data, io->bank, sizeof(write_data));
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_BANK_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK, write_data,
            joypad_n64_transfer_pak_store_write_callback, ctx
        );
    }
}

static void joypad_n64_transfer_pak_store_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];
    joypad_accessory_state_t state = accessory->state;
    if (!joypad_accessory_state_is_transfer_storing(state)) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joybus_callback_t retry_callback = joypad_n64_transfer_pak_store_write_callback;
    if (joypad_accessory_write_crc_error_check(port, recv_cmd, retry_callback, ctx))
    {
        return; // Accessory communication error!
    }
    volatile joypad_n64_transfer_pak_io_t *io = &accessory->transfer_pak_io;
    if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_BANK_WRITE)
    {
        // Continue writing data
        accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_DATA_WRITE;
        accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
        accessory->retries = 0;
        joybus_n64_accessory_write_async(
            port, io->tpak_addr, io->cursor,
            joypad_n64_transfer_pak_store_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_DATA_WRITE)
    {
        uint8_t *cursor = io->cursor += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        uint16_t tpak_addr = io->tpak_addr += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        uint16_t cart_addr = io->cart_addr += JOYBUS_N64_ACCESSORY_DATA_SIZE;
        int next_bank = cart_addr / JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
        if (cursor >= io->end)
        {
            // Finished writing data
            accessory->state = JOYPAD_ACCESSORY_STATE_IDLE;
        }
        else if (next_bank == io->bank)
        {
            // Continue writing data
            accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_DATA_WRITE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, tpak_addr, cursor,
                joypad_n64_transfer_pak_store_write_callback, ctx
            );
        }
        else
        {
            // Switch to the next bank
            io->tpak_addr = JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART;
            io->bank = next_bank;
            uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
            memset(write_data, next_bank, sizeof(write_data));
            accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_BANK_WRITE;
            accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
            accessory->retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_BANK, write_data,
                joypad_n64_transfer_pak_store_write_callback, ctx
            );
        }
    }
}

void joypad_n64_transfer_pak_store_async(joypad_port_t port, uint16_t cart_addr, void *src, size_t len)
{
    ASSERT_JOYBUS_CONTROLLER_PORT_VALID(port);
    assert(cart_addr % JOYBUS_N64_ACCESSORY_DATA_SIZE == 0);
    assert(len % JOYBUS_N64_ACCESSORY_DATA_SIZE == 0);
    volatile joypad_accessory_t *accessory = &joypad_accessories_hot[port];

    uint8_t bank = cart_addr / JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
    uint16_t tpak_addr = JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_CART;
    tpak_addr += cart_addr % JOYBUS_N64_ACCESSORY_TRANSFER_BANK_SIZE;
    accessory->transfer_pak_io = (joypad_n64_transfer_pak_io_t){
        .start = src,
        .end = src + len,
        .cursor = src,
        .bank = bank,
        .cart_addr = cart_addr,
        .tpak_addr = tpak_addr,
    };

    accessory->state = JOYPAD_ACCESSORY_STATE_TRANSFER_STORE_STATUS_READ;
    accessory->error = JOYPAD_ACCESSORY_ERROR_PENDING;
    accessory->retries = 0;
    joybus_n64_accessory_read_async(
        port, JOYBUS_N64_ACCESSORY_ADDR_TRANSFER_STATUS,
        joypad_n64_transfer_pak_store_read_callback, (void *)port
    );
}
