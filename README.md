# LibDragon Joypad Subsystem Test ROM

This is a simple test ROM for Nintendo 64 that demonstrates an abstraction concept to unify N64 and GameCube joypad input styles into a more-common interface.

![Screenshot of GameCube and N64 controller detected](./screenshot.png?raw=true)

## Supported Controllers

### Nintendo 64

The standard Nintendo 64 controller has fewer (and different) inputs than a GameCube controller:

* Missing X & Y buttons
* Missing a C analog stick; instead there are 4 digital C directional buttons
* Does not have any pressure sensitive buttons or triggers

### GameCube

GameCube controllers must be connected using a "passive adapter" that can be created by splicing the Joybus data pin on a GameCube controller plug into the Joybus data pin on the N64 console.

The GameCube controller has more (and different) inputs than a Nintendo 64 controller:

* Has X & Y buttons
* Has pressure-sensitive L & R triggers
* Missing C directional buttons; instead there is a C analog stick
* Some have pressure-sensitive A & B buttons
  * Analog A & B is non-functional on most GameCube controllers (official *and* third party)

## Compatibility compromises

* Nintendo 64 controllers cannot press X or Y buttons
* Nintendo 64 controllers (attempt to) emulate analog L & R triggers using digital inputs
* Nintendo 64 controllers (attempt to) emulate analog C stick using digital inputs
* GameCube controllers (attempt to) emulate C buttons based on analog C stick position
  * Players will not be able to press C-Left & C-Right or C-Up & C-Down simultaneously
* GameCube controllers always use analog mode 3 and will not read analog A & B inputs

## Rumble

If the controller has rumble capability, holding the A button will enable the rumble motor.

For Nintendo 64 controllers, this requires a Rumble Pak accessory.

For GameCube controllers, the 5-volt VCC pin on the controller plug must be connected to an external power source.

## Run the test ROM

[Download](./joypadtest.z64?raw=true) or [compile](#build-the-rom) the ROM file and load it as you would any other N64 ROM.

This ROM file has been tested to work on real Nintendo 64 hardware using the [EverDrive-64 by krikzz](http://krikzz.com/) and [64drive by retroactive](http://64drive.retroactive.be/).

## Build the ROM

1. [Install LibDragon](https://github.com/DragonMinded/libdragon) and make sure you export `N64_INST` as the path to your N64 compiler toolchain.
2. Run `make` to produce `joypadtest.z64`

## License

This project is [Unlicensed public domain software](./LICENSE.md?raw=true) written by [Christopher Bonhage](https://github.com/meeq).

LibDragon is [Unlicensed public domain software](https://github.com/DragonMinded/libdragon/blob/trunk/LICENSE.md?raw=true).

"Nintendo 64" and "GameCube" are registered trademarks of Nintendo used for informational purposes without permission.
