# LibDragon Joypad Subsystem Test ROM

This is a simple test ROM for Nintendo 64 that demonstrates an abstraction concept to unify N64 and GameCube joypad input styles into a more-common interface.

![Screenshot of GameCube and N64 controller detected](./screenshot.png?raw=true)

## Supported Controllers

### Nintendo 64

The standard Nintendo 64 controller has fewer (and different) inputs than a GameCube controller:

* Missing X & Y buttons
* Missing a C analog stick; instead there are 4 digital buttons
* L & R triggers are not pressure sensitive
* Analog stick values are in `(-128, 127)` format instead of `(0, 255)`

### GameCube

GameCube controllers must be connected using a "passive adapter" that can be created by splicing the Joybus data pin on a GameCube controller plug into the Joybus data pin on the N64 console.

The GameCube controller has more (and different) inputs than a Nintendo 64 controller:

* Has extra X & Y buttons
* Has pressure-sensitive L & R triggers
* Missing C buttons; instead there is a C analog stick
* Analog stick values are in `(0, 255)` format instead of `(-128, 127)`

## Compatibility compromises

* Analog stick values are normalized to "signed 8-bit" `(-128, 127)` format
* Analog triggers continue to be represented in "unsigned 8-bit" `(0, 255)` format
* Nintendo 64 Controllers cannot press X or Y buttons
* Nintendo 64 Controllers (badly) emulate analog L & R triggers using digital inputs
* Nintendo 64 Controllers (badly) emulate analog C stick using digial inputs
* GameCube Controllers (badly) emulate C buttons based on analog stick position

## Rumble

If the controller has rumble capability, holding the A button will enable the rumble motor.

For Nintendo 64 controllers, this requires a Rumble Pak accessory.

For GameCube controllers, the 5-volt VCC pin must be connected to an external power source for rumble to work.

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
