SRC =                          \
	src/bio_sensor.c           \
	src/joybus_n64_accessory.c \
	src/joypad_accessory.c     \
	src/joypad.c               \

all:                       \
	AccessoryProbeTest.z64 \
	AccessoryTest.z64      \
	BioSensorTest.z64      \
	ControllerPakDump.z64  \
	JoypadTest.z64         \
	TransferPakTest.z64    \

BUILD_DIR = build
include $(N64_INST)/include/n64.mk

N64_CFLAGS += -I./include

OBJS = $(SRC:%.c=$(BUILD_DIR)/%.o)
DEPS = $(SRC:%.c=$(BUILD_DIR)/%.d)

AccessoryProbeTest.z64: N64_ROM_TITLE = AccessoryProbeTest
$(BUILD_DIR)/AccessoryProbeTest.elf: $(OBJS) $(BUILD_DIR)/examples/AccessoryProbeTest.o

AccessoryTest.z64: N64_ROM_TITLE = AccessoryTest
$(BUILD_DIR)/AccessoryTest.elf: $(OBJS) $(BUILD_DIR)/examples/AccessoryTest.o

BioSensorTest.z64: N64_ROM_TITLE = BioSensorTest
$(BUILD_DIR)/BioSensorTest.elf: $(OBJS) $(BUILD_DIR)/examples/BioSensorTest.o

ControllerPakDump.z64: N64_ROM_TITLE = ControllerPakDump
ControllerPakDump.z64: N64_ROM_SAVETYPE = sram256k
$(BUILD_DIR)/ControllerPakDump.elf: $(OBJS) $(BUILD_DIR)/examples/ControllerPakDump.o

JoypadTest.z64: N64_ROM_TITLE = JoypadTest
$(BUILD_DIR)/JoypadTest.elf: $(OBJS) $(BUILD_DIR)/examples/JoypadTest.o

TransferPakTest.z64: N64_ROM_TITLE = TransferPakTest
$(BUILD_DIR)/TransferPakTest.elf: $(OBJS) $(BUILD_DIR)/examples/TransferPakTest.o

clean:
	rm -rf $(BUILD_DIR) *.z64
.PHONY: all clean

-include $(DEPS)
