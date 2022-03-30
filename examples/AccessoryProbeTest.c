/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <string.h>
#include <libdragon.h>

#include "joybus_n64_accessory.h"
#include "joypad.h"

#define PROBE_ADDR 0x8000
#define PROBE_BYTE 0x80

const uint8_t ALL_ZEROES[JOYBUS_N64_ACCESSORY_DATA_SIZE] = {0};
const uint8_t BIO_SENSOR_DETECTED[JOYBUS_N64_ACCESSORY_DATA_SIZE] = {
    [0 ... (JOYBUS_N64_ACCESSORY_DATA_SIZE - 1)] = JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR,
};

void print_data(uint8_t *data)
{
    for (size_t i = 0; i < JOYBUS_N64_ACCESSORY_DATA_SIZE; ++i)
    {
        if (i > 0 && i % 16 == 0) printf("\n");
        printf("%02X ", data[i]);
    }
    printf("\n");
}

int run_test(joypad_port_t port, uint8_t *data)
{
    int crc_status;
    printf("Writing to 0x%04X...\n", PROBE_ADDR);
    print_data(data);
    console_render();
    crc_status = joybus_n64_accessory_write(port, PROBE_ADDR, data);
    if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_DISCONNECTED)
    {
        printf("Accessory write disconnected.\n");
    }
    else if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH)
    {
        printf("Accessory write data CRC mismatch.\n");
    }
    else
    {
        printf("Reading from 0x%04X...\n", PROBE_ADDR);
        console_render();
        crc_status = joybus_n64_accessory_read(port, PROBE_ADDR, data);
        if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_DISCONNECTED)
        {
            printf("Accessory read disconnected.\n");
        }
        else if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH)
        {
            printf("Accessory read data CRC mismatch.\n");
        }
        else
        {
            print_data(data);
        }
    }
    console_render();
    return crc_status;
}

int run_tests(joypad_port_t port, uint8_t probe_byte)
{
    uint8_t data[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    uint8_t detected[JOYBUS_N64_ACCESSORY_DATA_SIZE];
    memset(detected, probe_byte, sizeof(detected));
    int crc_status;

    printf("Test 1: Set all bytes to 0x%02X\n", probe_byte);
    memset((void *)data, probe_byte, sizeof(data));
    crc_status = run_test(port, data);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK) return 1;
    // Test 1 result analysis
    if (
        probe_byte != JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR &&
        memcmp(data, BIO_SENSOR_DETECTED, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0
    )
    {
        // Bio Sensor always reports itself regardless of probe write value
        printf("Bio Sensor detected!\n");
    }
    else if (
        probe_byte == JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF ||
        probe_byte == JOYBUS_N64_ACCESSORY_PROBE_CONTROLLER_PAK
    )
    {
        if (memcmp(data, ALL_ZEROES, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory read result matches expectations.\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    else
    {
        if (memcmp(data, detected, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory successfully detected!\n");
        }
        else if (memcmp(data, ALL_ZEROES, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory not detected!\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    console_render();

    printf("Test 2: Only set the last byte to 0x%02X\n", probe_byte);
    memset((void *)data, 0, sizeof(data));
    data[sizeof(data) - 1] = probe_byte;
    crc_status = run_test(port, data);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK) return 1;
    // Test 2 result analysis
    if (
        probe_byte != JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR &&
        memcmp(data, BIO_SENSOR_DETECTED, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0
    )
    {
        // Bio Sensor always reports itself regardless of probe write value
        printf("Bio Sensor detected!\n");
    }
    else if (
        probe_byte == JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF ||
        probe_byte == JOYBUS_N64_ACCESSORY_PROBE_CONTROLLER_PAK
    )
    {
        if (memcmp(data, ALL_ZEROES, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory read result matches expectations.\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    else
    {
        if (memcmp(data, detected, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory successfully detected!\n");
        }
        else if (memcmp(data, ALL_ZEROES, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory not detected!\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    console_render();

    printf("Test 3: Only set the first byte to 0x%02X\n", probe_byte);
    memset((void *)data, 0, sizeof(data));
    data[0] = probe_byte;
    crc_status = run_test(port, data);
    if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK) return 1;
    // Test 3 result analysis
    if (probe_byte == JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR)
    {
        // Bio Sensor should report itself regardless of probe write value
        if (memcmp(data, detected, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory read result matches expectations.\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    else
    {
        if (memcmp(data, ALL_ZEROES, JOYBUS_N64_ACCESSORY_DATA_SIZE) == 0)
        {
            printf("Accessory read result matches expectations.\n");
        }
        else
        {
            printf("Unexpected data in accessory read result!\n");
        }
    }
    console_render();

    return 0;
}

void loop_tests(joypad_port_t port, uint8_t probe_byte)
{
    joypad_buttons_t p1;
    int crc_status;

    while (1)
    {
        console_clear();
        printf("Probing Accessory on Controller Port %d...\n\n", port + 1);
        crc_status = run_tests(port, probe_byte);
        if (crc_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK)
        {
            printf("Tests ran OK");
        }
        else
        {
            printf("Tests failed");
        }
        printf("; press A to retry or B to go back");
        console_render();

        while (1)
        {
            joypad_read();
            p1 = joypad_get_buttons_pressed(JOYPAD_PORT_1);
            if (p1.a) break;
            if (p1.b) return;
        }
    }
}

int main(void)
{
    timer_init();
    display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);

    console_init();
    console_set_render_mode(RENDER_MANUAL);
    console_set_debug(false);

    joypad_init();
    joypad_buttons_t p1;
    joypad_port_t port = JOYPAD_PORT_1;

    while (1)
    {
        console_clear();
        printf("N64 Accessory Probe Tester v0.5 by Meeq\n");
        printf("\n");
        printf("These tests attempt to detect various controller accessories\n");
        printf("by writing magic values to 0x8000 and reading them back.\n");
        printf("\n");
        printf("Test 1 and Test 2 should produce the same result.\n");
        printf("Test 3 results should read all bytes as 00 (except Bio Sensor).\n");
        printf("\n");
        printf("If the Pak you are trying to detect is connected, the read\n");
        printf("buffer should have all bytes set to the probe value.\n");
        printf("\n");
        printf("If a different Pak than the one you are trying to detect is\n");
        printf("connected, the read buffer should have all bytes as 00.\n");
        printf("\n");
        printf("Controller Pak should read all bytes as 00 for all tests.\n");
        printf("Transfer Pak OFF should read all bytes as 00 for all tests.\n");
        printf("Bio Sensor should read all bytes as 81 for all tests.\n");
        printf("\n");
        printf("Press C-Left  to detect Rumble Pak           (0x80)\n");
        printf("Press C-Right to detect Transfer Pak         (0x84)\n");
        printf("Press C-Up    to detect Bio Sensor           (0x81)\n");
        printf("Press C-Down  to detect Pokemon Snap Station (0x85)\n");
        printf("Press B       to turn Transfer Pak OFF       (0xFE)\n");
        printf("Press A       to probe with all zeroes       (0x00)\n");
        printf("\n");
        printf("Tests will access accessories on Controller Port %d\n", port + 1);
        printf("Press L to decrement or R to increment Controller Port.\n");
        console_render();

        joypad_read();
        p1 = joypad_get_buttons_pressed(JOYPAD_PORT_1);
        // Port selection
        if (p1.l) port = port <= JOYPAD_PORT_1 ? JOYPAD_PORT_4 : port - 1;
        if (p1.r) port = port >= JOYPAD_PORT_4 ? JOYPAD_PORT_1 : port + 1;
        // Test selection
             if (p1.a)       loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_CONTROLLER_PAK);
        else if (p1.b)       loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_OFF);
        else if (p1.c_left)  loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_RUMBLE_PAK);
        else if (p1.c_right) loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_TRANSFER_PAK_ON);
        else if (p1.c_up)    loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_BIO_SENSOR);
        else if (p1.c_down)  loop_tests(port, JOYBUS_N64_ACCESSORY_PROBE_SNAP_STATION);
    }
}
