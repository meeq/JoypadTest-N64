/**
 * @file main.c
 * @author Christopher Bonhage (me@christopherbonhage.com)
 * @brief N64 test ROM for Joypad subsystem
 */

#include <assert.h>
#include <string.h>
#include <libdragon.h>

#include "joybus_n64_accessory.h"
#include "joypad.h"

#define CONTROLLER_PAK_SIZE  0x8000
#define SRAM_256KBIT_SIZE    0x8000

#define CART_DOM2_ADDR2_START  0x08000000
#define CART_DOM2_ADDR2_SIZE   0x08000000

typedef struct PI_regs_s {
    volatile void * ram_address;
    uint32_t pi_address;
    uint32_t read_length;
    uint32_t write_length;
} PI_regs_t;
static volatile PI_regs_t * const PI_regs = (PI_regs_t *)0xA4600000;

void cart_dom2_addr2_read(void * dest, uint32_t offset, uint32_t len)
{
    assert(dest != NULL);
    assert(offset < CART_DOM2_ADDR2_SIZE);
    assert(len > 0);
    assert(offset + len <= CART_DOM2_ADDR2_SIZE);

    disable_interrupts();
    dma_wait();

    MEMORY_BARRIER();
    PI_regs->ram_address = UncachedAddr(dest);
    MEMORY_BARRIER();
    PI_regs->pi_address = offset | CART_DOM2_ADDR2_START;
    MEMORY_BARRIER();
    PI_regs->write_length = len - 1;
    MEMORY_BARRIER();

    enable_interrupts();
    dma_wait();
}

void cart_dom2_addr2_write(const void * src, uint32_t offset, uint32_t len)
{
    assert(src != NULL);
    assert(offset < CART_DOM2_ADDR2_SIZE);
    assert(len > 0);
    assert(offset + len <= CART_DOM2_ADDR2_SIZE);

    disable_interrupts();
    dma_wait();

    MEMORY_BARRIER();
    PI_regs->ram_address = UncachedAddr(src);
    MEMORY_BARRIER();
    PI_regs->pi_address = offset | CART_DOM2_ADDR2_START;
    MEMORY_BARRIER();
    PI_regs->read_length = len - 1;
    MEMORY_BARRIER();

    enable_interrupts();
    dma_wait();
}

bool cart_sram_verify(void)
{
    uint8_t __attribute__ ((aligned(16))) write_buf[SRAM_256KBIT_SIZE];
    uint8_t __attribute__ ((aligned(16))) read_buf[SRAM_256KBIT_SIZE];

    /* Generate test values based on the destination SRAM addresses */
    uint32_t * write_words = (uint32_t *)write_buf;
    for (size_t i = 0; i < SRAM_256KBIT_SIZE / sizeof(uint32_t); ++i)
    {
        write_words[i] = i;
    }

    /* Write the test values into SRAM */
    data_cache_hit_writeback_invalidate(write_buf, SRAM_256KBIT_SIZE);
    cart_dom2_addr2_write(write_buf, 0, SRAM_256KBIT_SIZE);

    /* Read the test values back to see if they persisted */
    data_cache_hit_writeback_invalidate(read_buf, SRAM_256KBIT_SIZE);
    cart_dom2_addr2_read(read_buf, 0, SRAM_256KBIT_SIZE);

    /* Compare what was written to what was read back from SRAM */
    if (memcmp(write_buf, read_buf, SRAM_256KBIT_SIZE ) != 0)
    {
        /* There was a mismatch between what was written and read */
        return false;
    }

    /* Erase SRAM */
    memset(write_buf, 0, SRAM_256KBIT_SIZE);
    data_cache_hit_writeback_invalidate( write_buf, SRAM_256KBIT_SIZE);
    cart_dom2_addr2_write(write_buf, 0, SRAM_256KBIT_SIZE);

    return true;
}

int dump_controller_pak(joypad_port_t port)
{
    int crc_status;
    uint8_t __attribute__((aligned(16))) dump[CONTROLLER_PAK_SIZE];

    for (size_t addr = 0; addr < CONTROLLER_PAK_SIZE; addr += JOYBUS_N64_ACCESSORY_DATA_SIZE)
    {
        crc_status = joybus_n64_accessory_read_sync(port, addr, &dump[addr]);
        if (crc_status != JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_OK) return crc_status;
        printf(".");
        console_render();
    }
    printf("\nWriting to SRAM...\n");

    data_cache_hit_writeback_invalidate(dump, sizeof(dump));
    cart_dom2_addr2_write(dump, 0, sizeof(dump));

    return crc_status;
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
    joypad_accessory_type_t accessory_type;
    int dump_status;

    bool sram_detected = cart_sram_verify();

    while (1)
    {
        joypad_scan();
        p1 = joypad_get_buttons_pressed(port);
        accessory_type = joypad_get_accessory_type(port);

        console_clear();
        printf("N64 Controller Pak Dump v3 by Meeq\n");
        printf("\n");

        if (!sram_detected)
        {
            printf("Cartridge SRAM not detected.\n");
            printf("Check your flashcart settings!\n");
            console_render();
        }
        else if (accessory_type != JOYPAD_ACCESSORY_TYPE_CONTROLLER_PAK)
        {
            printf("Insert a Controller Pak in Controller Port 1 to continue\n");
            console_render();
        }
        else
        {
            printf("Press A to dump the Controller Pak into Cartridge SRAM\n");
            console_render();
            if (p1.a)
            {
                printf("Dumping...\n");
                console_render();
                dump_status = dump_controller_pak(port);
                if (dump_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_NO_PAK)
                {
                    printf("Dump failed due to accessory disconnect!\n");
                }
                else if (dump_status == JOYBUS_N64_ACCESSORY_DATA_CRC_STATUS_MISMATCH)
                {
                    printf("Dump failed due to data CRC mismatch!\n");
                }
                else
                {
                    printf("Dump complete!\n");
                }
                printf("Press B to continue\n");
                console_render();
                while (1)
                {
                    joypad_scan();
                    p1 = joypad_get_buttons_pressed(port);
                    if (p1.b) break; 
                }
            }
        }
    }
}
