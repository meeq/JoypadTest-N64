all: roms

SRC =                          \
	src/bio_sensor.c           \
	src/joybus_n64_accessory.c \
	src/joypad_accessory.c     \
	src/joypad.c               \

OBJS = $(SRC:%.c=$(BUILD_DIR)/%.o)
DEPS = $(SRC:%.c=$(BUILD_DIR)/%.d)

BUILD_DIR = build
include $(N64_INST)/include/n64.mk

N64_CFLAGS += -I./include

# Test ROM versions (bump when distributing)
ACCESSORY_PROBE_TEST_VERSION = v5
BIO_SENSOR_TEST_VERSION      = v4
CONTROLLER_PAK_DUMP_VERSION  = v2
GB_CAMERA_TEST_VERSION       = v1
RUMBLE_SHORT_TEST_VERSION    = v1
SNAP_STATION_TEST_VERSION    = v4
TRANSFER_PAK_TEST_VERSION    = v2

# Test ROMs

roms:                      \
	AccessoryProbeTest.z64 \
	RumbleShortTest.z64 \
	AccessoryTest.z64      \
	BioSensorTest.z64      \
	ControllerPakDump.z64  \
	GBCameraTest.z64       \
	JoypadTest.z64         \
	SnapStationTest.z64    \
	TransferPakTest.z64    \

AccessoryProbeTest.z64: N64_ROM_TITLE = AccessoryProbeTest
AccessoryProbeTest.z64: N64_CFLAGS += -DROM_VERSION='"$(ACCESSORY_PROBE_TEST_VERSION)"'
$(BUILD_DIR)/AccessoryProbeTest.elf: $(OBJS) $(BUILD_DIR)/examples/AccessoryProbeTest.o

AccessoryTest.z64: N64_ROM_TITLE = AccessoryTest
$(BUILD_DIR)/AccessoryTest.elf: $(OBJS) $(BUILD_DIR)/examples/AccessoryTest.o

BioSensorTest.z64: N64_ROM_TITLE = BioSensorTest
BioSensorTest.z64: N64_CFLAGS += -DROM_VERSION='"$(BIO_SENSOR_TEST_VERSION)"'
$(BUILD_DIR)/BioSensorTest.elf: $(OBJS) $(BUILD_DIR)/examples/BioSensorTest.o

ControllerPakDump.z64: N64_ROM_TITLE = ControllerPakDump
ControllerPakDump.z64: N64_ROM_SAVETYPE = sram256k
ControllerPakDump.z64: N64_CFLAGS += -DROM_VERSION='"$(CONTROLLER_PAK_DUMP_VERSION)"'
$(BUILD_DIR)/ControllerPakDump.elf: $(OBJS) $(BUILD_DIR)/examples/ControllerPakDump.o

GBCameraTest.z64: N64_ROM_TITLE = GBCameraTest
GBCameraTest.z64: N64_CFLAGS += -DROM_VERSION='"$(GB_CAMERA_TEST_VERSION)"'
$(BUILD_DIR)/GBCameraTest.elf: $(OBJS) $(BUILD_DIR)/examples/GBCameraTest.o

JoypadTest.z64: N64_ROM_TITLE = JoypadTest
$(BUILD_DIR)/JoypadTest.elf: $(OBJS) $(BUILD_DIR)/examples/JoypadTest.o

RumbleShortTest.z64: N64_ROM_TITLE = RumbleShortTest
RumbleShortTest.z64: N64_CFLAGS += -DROM_VERSION='"$(RUMBLE_SHORT_TEST_VERSION)"'
$(BUILD_DIR)/RumbleShortTest.elf: $(OBJS) $(BUILD_DIR)/examples/RumbleShortTest.o

SnapStationTest.z64: N64_ROM_TITLE = SnapStationTest
SnapStationTest.z64: N64_CFLAGS += -DROM_VERSION='"$(SNAP_STATION_TEST_VERSION)"'
$(BUILD_DIR)/SnapStationTest.elf: $(OBJS) $(BUILD_DIR)/examples/SnapStationTest.o

TransferPakTest.z64: N64_ROM_TITLE = TransferPakTest
TransferPakTest.z64: N64_CFLAGS += -DROM_VERSION='"$(TRANSFER_PAK_TEST_VERSION)"'
$(BUILD_DIR)/TransferPakTest.elf: $(OBJS) $(BUILD_DIR)/examples/TransferPakTest.o

# Archives

archives:                                                  \
	AccessoryProbeTest-$(ACCESSORY_PROBE_TEST_VERSION).zip \
	BioSensorTest-$(BIO_SENSOR_TEST_VERSION).zip           \
	ControllerPakDump-$(CONTROLLER_PAK_DUMP_VERSION).zip   \
	GBCameraTest-$(GB_CAMERA_TEST_VERSION).zip             \
	RumbleShortTest-$(RUMBLE_SHORT_TEST_VERSION).zip       \
	SnapStationTest-$(SNAP_STATION_TEST_VERSION).zip       \
	TransferPakTest-$(TRANSFER_PAK_TEST_VERSION).zip       \

AccessoryProbeTest-$(ACCESSORY_PROBE_TEST_VERSION).zip: AccessoryProbeTest.z64
	cp $^ $(^:%.z64=%-$(ACCESSORY_PROBE_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(ACCESSORY_PROBE_TEST_VERSION).z64)

BioSensorTest-$(BIO_SENSOR_TEST_VERSION).zip: BioSensorTest.z64
	cp $^ $(^:%.z64=%-$(BIO_SENSOR_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(BIO_SENSOR_TEST_VERSION).z64)

ControllerPakDump-$(CONTROLLER_PAK_DUMP_VERSION).zip: ControllerPakDump.z64
	cp $^ $(^:%.z64=%-$(CONTROLLER_PAK_DUMP_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(CONTROLLER_PAK_DUMP_VERSION).z64)

GBCameraTest-$(GB_CAMERA_TEST_VERSION).zip: GBCameraTest.z64
	cp $^ $(^:%.z64=%-$(GB_CAMERA_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(GB_CAMERA_TEST_VERSION).z64)

RumbleShortTest-$(RUMBLE_SHORT_TEST_VERSION).zip: RumbleShortTest.z64
	cp $^ $(^:%.z64=%-$(RUMBLE_SHORT_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(RUMBLE_SHORT_TEST_VERSION).z64)

SnapStationTest-$(SNAP_STATION_TEST_VERSION).zip: SnapStationTest.z64
	cp $^ $(^:%.z64=%-$(SNAP_STATION_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(SNAP_STATION_TEST_VERSION).z64)

TransferPakTest-$(TRANSFER_PAK_TEST_VERSION).zip: TransferPakTest.z64
	cp $^ $(^:%.z64=%-$(TRANSFER_PAK_TEST_VERSION).z64)
	zip -m $@ $(^:%.z64=%-$(TRANSFER_PAK_TEST_VERSION).z64)

# Housekeeping

clean:
	rm -rf $(BUILD_DIR) *.z64 *.zip

-include $(DEPS)

.PHONY: all archives clean roms
