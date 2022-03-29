# LibDragon Bio Sensor Subsystem Test ROM

This is a simple test ROM for Nintendo 64 that demonstrates how to read the pulse data from the Bio Sensor accessory used by Tetris 64. It allows reading on pulse rate on all four controller ports.

Press A to start reading the Bio Sensor accessory. The subsystem reads from each port once per VI interrupt, which should be sufficient to keep up with normal human pulse rates.

Press B to stop reading the Bio Sensor accessory. If the accessory is disconnected, the subsystem will automatically stop reading the port.

## Run the test ROM

[Download](./BioSensorTest.z64?raw=true) or [compile](#build-the-rom) the ROM file and load it as you would any other N64 ROM.

This ROM file has been tested to work on real Nintendo 64 hardware using the [EverDrive-64 by krikzz](http://krikzz.com/) and [64drive by retroactive](http://64drive.retroactive.be/).

## Build the ROM

1. [Install LibDragon](https://github.com/DragonMinded/libdragon) and make sure you export `N64_INST` as the path to your N64 compiler toolchain.
2. Run `make` to produce `BioSensorTest.z64`

## License

This project is [Unlicensed public domain software](./LICENSE.md?raw=true) written by [Christopher Bonhage](https://github.com/meeq).

LibDragon is [Unlicensed public domain software](https://github.com/DragonMinded/libdragon/blob/trunk/LICENSE.md?raw=true).

"Nintendo 64" is a registered trademarks of Nintendo used for informational purposes without permission.
