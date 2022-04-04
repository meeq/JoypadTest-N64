
/**
 * @file joypad_accessory.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief Joypad Accessory helpers
 * @ingroup joypad
 */

#include <string.h>

#include "joybus_n64_accessory.h"
#include "joypad_accessory.h"
#include "joypad_internal.h"

static void joypad_accessory_detect_read_callback(uint64_t *out_dwords, void *ctx);
static void joypad_accessory_detect_write_callback(uint64_t *out_dwords, void *ctx);

static joypad_accessory_error_t joypad_accessory_crc_error_check(
    volatile joypad_device_hot_t *device,
    const uint8_t *data, uint8_t data_crc
)
{
    switch (joybus_n64_accessory_data_crc_compare(data, data_crc))
    {
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK:
        {
            // Accessory is no longer connected!
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_NONE;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_NONE;
            device->rumble_active = false;
            device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
            device->accessory.status = JOYBUS_IDENTIFY_STATUS_N64_ACCESSORY_ABSENT;
            return device->accessory.error = JOYPAD_ACCESSORY_ERROR_ABSENT;
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH:
        {
            size_t retries = device->accessory.retries;
            if (retries < JOYPAD_ACCESSORY_RETRY_LIMIT)
            {
                device->accessory.retries = retries + 1;
                return device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
            }
            else
            {
                device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
                return device->accessory.error = JOYPAD_ACCESSORY_ERROR_CHECKSUM;
            }
        }
        case JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK:
        default:
        {
            return device->accessory.error = JOYPAD_ACCESSORY_ERROR_NONE;
        }
    }
}

static void joypad_accessory_detect_read_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_accessory_state_t state = device->accessory.state;
    if (!joypad_accessory_state_is_detecting(state)) return;

    uint8_t write_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    const joybus_cmd_n64_accessory_read_port_t *recv_cmd = (void *)&out_bytes[port];
    joypad_accessory_error_t read_error = joypad_accessory_crc_error_check(
        device, recv_cmd->data, recv_cmd->data_crc
    );
    if (read_error)
    {
        if (read_error == JOYPAD_ACCESSORY_ERROR_PENDING)
        {
            // Retry: Bad communication with the accessory
            uint16_t retry_addr = recv_cmd->addr_checksum;
            retry_addr &= JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET;
            joybus_n64_accessory_read_async(
                port, retry_addr,
                joypad_accessory_detect_read_callback, ctx
            ); 
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ)
    {
        // Compare the expected label with what was actually read back
        for (size_t i = 0; i < sizeof(write_data); ++i) write_data[i] = i;
        if (memcmp(recv_cmd->data, write_data, sizeof(write_data)) == 0)
        {
            // Detect: Label write persisted; this appears to be a Controller Pak
            device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_CONTROLLER_PAK;
        }
        else if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR)
        {
            // Detect: Bio Sensor responds to all reads with probe value
            device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_BIO_SENSOR;
        }
        else if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK)
        {
            // Detect: Some emulators respond to all Rumble Pak reads with probe value
            device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_RUMBLE_PAK;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK;
        }
        else
        {
            // Step 3A: Write probe value to detect Rumble Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK, sizeof(write_data));
            device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_WRITE;
            device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
            device->accessory.retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_READ)
    {
        if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK)
        {
            // Detect: Probe reports that this is a Rumble Pak
            device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_RUMBLE_PAK;
            device->rumble_method = JOYPAD_RUMBLE_METHOD_N64_RUMBLE_PAK;
        }
        else
        {
            // Step 4A: Write probe value to detect Transfer Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON, sizeof(write_data));
            device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_ON;
            device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
            device->accessory.retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_READ)
    {
        if (recv_cmd->data[0] == JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON)
        {
            // Step 4C: Probe reports that this is a Transfer Pak; turn off Transfer Pak
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF, sizeof(write_data));
            device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_OFF;
            device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
            device->accessory.retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
        else
        {
            // Step 5A: Write probe value to detect Pokemon Snap Station
            memset(write_data, JOYBUS_N64_ACCESSORY_PROBE_SNAP_STATION, sizeof(write_data));
            device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_SNAP_WRITE;
            device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
            device->accessory.retries = 0;
            joybus_n64_accessory_write_async(
                port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, write_data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_SNAP_READ)
    {
        if (recv_cmd->data[0] == JOYPAD_ACCESSORY_TYPE_SNAP_STATION)
        {
            // Detect: Probe reports that this is a Snap Station
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_SNAP_STATION;
        }
        else
        {
            // Failure: Unable to determine which accessory is connected
            device->accessory.type = JOYPAD_ACCESSORY_TYPE_UNKNOWN;
        }
        device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
    }
}

static void joypad_accessory_detect_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_accessory_state_t state = device->accessory.state;
    if (!joypad_accessory_state_is_detecting(state)) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joypad_accessory_error_t write_error = joypad_accessory_crc_error_check(
        device, recv_cmd->data, recv_cmd->data_crc
    );
    if (write_error)
    {
        if (write_error == JOYPAD_ACCESSORY_ERROR_PENDING)
        {
            // Retry: Bad communication with the accessory
            uint16_t retry_addr = recv_cmd->addr_checksum;
            retry_addr &= JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET;
            joybus_n64_accessory_write_async(
                port, retry_addr, recv_cmd->data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_INIT)
    {
        // Step 2A: Overwrite "label" area to detect Controller Pak
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        for (size_t i = 0; i < sizeof(data); ++i) data[i] = i;
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_LABEL, data,
            joypad_accessory_detect_write_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_LABEL_WRITE)
    {
        // Step 2B: Read back the "label" area to detect Controller Pak
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_LABEL_READ;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_LABEL,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_WRITE)
    {
        // Step 3B: Read probe value to detect Rumble Pak
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_RUMBLE_READ;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_ON)
    {
        // Step 4B: Read probe value to detect Transfer Pak
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_READ;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE,
            joypad_accessory_detect_read_callback, ctx
        );
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_TRANSFER_OFF)
    {
        // Detect: Transfer Pak has been probed and powered off
        device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
        device->accessory.type = JOYPAD_ACCESSORY_TYPE_TRANSFER_PAK;
    }
    else if (state == JOYPAD_ACCESSORY_STATE_DETECT_SNAP_WRITE)
    {
        // Step 5B: Read probe value to detect Pokemon Snap Station
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_SNAP_READ;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        joybus_n64_accessory_read_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE,
            joypad_accessory_detect_read_callback, ctx
        );
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
 * Step 4C: Turn off Transfer Pak if detected
 * Step 5A: Write probe value to detect Pokemon Snap Station
 * Step 5B: Read probe value to detect Pokemon Snap Station
 * 
 * @param[in] port Which controller port to detect the accessory on
 */
void joypad_accessory_detect_async(joypad_port_t port)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    if (device->accessory.state == JOYPAD_ACCESSORY_STATE_IDLE)
    {
        // Step 1: Ensure Transfer Pak is turned off
        uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
        memset(data, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF, sizeof(data));
        device->accessory.state = JOYPAD_ACCESSORY_STATE_DETECT_INIT;
        device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
        device->accessory.retries = 0;
        void *ctx = (void *)port;
        joybus_n64_accessory_write_async(
            port, JOYBUS_N64_ACCESSORY_ADDR_PROBE, data,
            joypad_accessory_detect_write_callback, ctx
        );
    }
}

static void joypad_n64_rumble_pak_toggle_write_callback(uint64_t *out_dwords, void *ctx)
{
    const uint8_t *out_bytes = (void *)out_dwords;
    joypad_port_t port = (joypad_port_t)ctx;
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    joypad_accessory_state_t state = device->accessory.state;
    if (state != JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE) return;

    const joybus_cmd_n64_accessory_write_port_t *recv_cmd = (void *)&out_bytes[port];
    joypad_accessory_error_t write_error = joypad_accessory_crc_error_check(
        device, recv_cmd->data, recv_cmd->data_crc
    );
    if (write_error)
    {
        if (write_error == JOYPAD_ACCESSORY_ERROR_PENDING)
        {
            // Retrying due to communication error
            uint16_t retry_addr = recv_cmd->addr_checksum;
            retry_addr &= JOYBUS_N64_ACCESSORY_ADDR_MASK_OFFSET;
            joybus_n64_accessory_write_async(
                port, retry_addr, recv_cmd->data,
                joypad_accessory_detect_write_callback, ctx
            );
        }
    }
    else
    {
        device->accessory.state = JOYPAD_ACCESSORY_STATE_IDLE;
    }
}

void joypad_n64_rumble_pak_toggle_async(joypad_port_t port, bool active)
{
    volatile joypad_device_hot_t *device = &joypad_devices_hot[port];
    device->accessory.state = JOYPAD_ACCESSORY_STATE_RUMBLE_WRITE;
    device->accessory.error = JOYPAD_ACCESSORY_ERROR_PENDING;
    device->accessory.retries = 0;
    device->rumble_active = active;
    uint8_t motor_data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(motor_data, active, sizeof(motor_data));
    joybus_n64_accessory_write_async(
        port, JOYBUS_N64_ACCESSORY_ADDR_RUMBLE, motor_data,
        joypad_n64_rumble_pak_toggle_write_callback, (void *)port
    );
}
