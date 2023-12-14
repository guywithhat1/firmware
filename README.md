# lib-source
This branch contains all the libraries the firmware uses, including Teensy base code.

# Documentation
Used Libraries:
 * [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
 * [Adafruit ICM20X](https://github.com/adafruit/Adafruit_ICM20X)
 * [Adafruit LIS3MDL](https://github.com/adafruit/Adafruit_LIS3MDL)
 * [Adafruit LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS)
 * [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
 * [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4)
    * NOTE: this is used effectively as a header only library
 * [FreqMeasureMulti](https://github.com/PaulStoffregen/FreqMeasureMulti)
 * [SPI](https://github.com/arduino/ArduinoCore-avr/tree/master/libraries/SPI)
 * [unity](https://github.com/ThrowTheSwitch/Unity)
    * NOTE: only select files used
 * [Wire](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
    * NOTE: we use a custom version made by Paul

## Installation & Usage
All that is needed to compile these into a static library is to run:
    `make`


## Contributing
In order to add a library to this, add the library to the `libraries` folder. All of the source files must be in the base level of this folder.

Next, you need to add your library to the makefile in two places.
 * In LIBRARY_SOURCE, append this value: `libraries/[your lib name]/*.cpp`
 * In LIBRARY_INCLUDE, append this value: `-Ilibraries/[your lib name]`

Running `make` should compile correctly if your source and header files are in the base directory of libraries/[your lib name]. 

NOTE: you may have header files nested, as long as any files that use those headers #include them as such. 