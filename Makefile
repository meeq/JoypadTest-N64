all: BioSensorTest.z64 JoypadTest.z64
.PHONY: all

BUILD_DIR = build
include $(N64_INST)/include/n64.mk

N64_CFLAGS += -I./include

SRC = \
	src/joypad.c \
	src/joybus_n64_accessory.c \
	src/bio_sensor.c \

OBJS = $(SRC:%.c=$(BUILD_DIR)/%.o)
DEPS = $(SRC:%.c=$(BUILD_DIR)/%.d)

JoypadTest.z64: N64_ROM_TITLE = "Joypad Test"
$(BUILD_DIR)/JoypadTest.elf: $(OBJS) $(BUILD_DIR)/examples/JoypadTest.o

BioSensorTest.z64: N64_ROM_TITLE = "Bio Sensor Test"
$(BUILD_DIR)/BioSensorTest.elf: $(OBJS) $(BUILD_DIR)/examples/BioSensorTest.o

clean:
	rm -rf $(BUILD_DIR) *.z64
.PHONY: clean

-include $(DEPS)
