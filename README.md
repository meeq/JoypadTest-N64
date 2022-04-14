# LibDragon Joypad Subsystem

This is a working proposal for LibDragon to adopt a new an abstraction concept to unify N64 and GameCube joypad input styles into a more-common interface.

Additionally, this subsystem intends to replace the usage of synchronous Joybus commands with an asynchronous model so that games aren't burning CPU cycles waiting on a slow serial interface.

## Joypad Subsystem

### Supported Controllers

#### Nintendo 64

The standard Nintendo 64 controller has fewer (and different) inputs than a GameCube controller:

* Missing X & Y buttons
* Missing a C analog stick; instead there are 4 digital C directional buttons
* Does not have any pressure sensitive buttons or triggers

#### GameCube

GameCube controllers must be connected using a "passive adapter" that can be created by splicing the Joybus data pin on a GameCube controller plug into the Joybus data pin on the N64 console.

The GameCube controller has more (and different) inputs than a Nintendo 64 controller:

* Has X & Y buttons
* Has pressure-sensitive L & R triggers
* Missing C directional buttons; instead there is a C analog stick
* Some have pressure-sensitive A & B buttons
  * Analog A & B is non-functional on most GameCube controllers (official *and* third party)

### Compatibility compromises

* Nintendo 64 controllers cannot press X or Y buttons
* Nintendo 64 controllers (attempt to) emulate analog L & R triggers using digital inputs
* Nintendo 64 controllers (attempt to) emulate analog C stick using digital inputs
* GameCube controllers (attempt to) emulate C buttons based on analog C stick position
  * Players will not be able to press C-Left & C-Right or C-Up & C-Down simultaneously
* GameCube controllers always use analog mode 3 and will not read analog A & B inputs

### Rumble

If the controller has rumble capability, holding the A button will enable the rumble motor.

For Nintendo 64 controllers, this requires a Rumble Pak accessory.

For GameCube controllers, the 5-volt VCC pin on the controller plug must be connected to an external power source.

## Build the project

1. [Install LibDragon](https://github.com/DragonMinded/libdragon) and make sure you export `N64_INST` as the path to your N64 compiler toolchain.
2. Run `make` to produce the test ROMs

## Example ROMs

### Accessory Probe Test

Test commands that attempt to detect various controller accessories by writing magic values to `0x8000` and reading them back.

### Bio Sensor Subsystem Test

This is a simple test ROM for Nintendo 64 that demonstrates how to read the pulse data from the Bio Sensor accessory used by Tetris 64. It allows reading on pulse rate on all four controller ports.

Press A to start reading the Bio Sensor accessory. The subsystem reads from each port once per VI interrupt, which should be sufficient to keep up with normal human pulse rates.

Press B to stop reading the Bio Sensor accessory. If the accessory is disconnected, the subsystem will automatically stop reading the port.

### Controller Pak Dump

Press A to dump the contents of the Controller Pak in port 1 to cartridge SRAM.

You may have to enable SRAM for this ROM in your flashcart settings. Once finished, you can copy the
SRAM data off the flashcart SD card.

### Snap Station Test

If you actually have a Pokemon Snap Station, this test ROM will allow you to exercise the various commands used by Pokemon Snap and Pokemon Stadium to print a sheet of stickers.

### Transfer Pak Test

Press A to enable the Transfer Pak. The Transfer Pak status flags will be displayed after enabling.

Press B to disable the Transfer Pak. Once the Transfer Pak status flags update, it is safe to remove the Game Boy cartridge.

Press C-Left to read the Game Boy cartridge ROM header.

Press C-Up to start the rumble motor on a Game Boy cartridge that uses the MBC5 mapper chip and supports rumble.

Press C-Down to stop the rumble motor.

## License

This project is [Unlicensed public domain software](./LICENSE.md?raw=true) written by [Christopher Bonhage](https://github.com/meeq).

LibDragon is [Unlicensed public domain software](https://github.com/DragonMinded/libdragon/blob/trunk/LICENSE.md?raw=true).

"Nintendo 64" is a registered trademarks of Nintendo used for informational purposes without permission.
