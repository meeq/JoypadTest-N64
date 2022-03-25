all: BioSensorTest.z64
.PHONY: all

BUILD_DIR = build
include $(N64_INST)/include/n64.mk

SRC = main.c joypad.c joybus_n64_accessory.c bio_sensor.c
OBJS = $(SRC:%.c=$(BUILD_DIR)/%.o)
DEPS = $(SRC:%.c=$(BUILD_DIR)/%.d)

BioSensorTest.z64: N64_ROM_TITLE = "Bio Sensor Test"

$(BUILD_DIR)/BioSensorTest.elf: $(OBJS)

clean:
	rm -rf $(BUILD_DIR) *.z64
.PHONY: clean

-include $(DEPS)
