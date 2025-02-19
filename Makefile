# Created by Jackson Stepka (Github: Pandabear1125)

# Use uname to detect current OS
UNAME := $(shell uname -s)

# Teensy core library files
TEENSY_DIR = teensy4
TEENSY_INCLUDE = -I$(TEENSY_DIR)
# name of the output lib file
TEENSY_LIB_NAME = libteensy4.a
# lib file name stripped of initial 'lib' and '.a'
TEENSY_LIB = teensy4

# Used external libraries
LIBRARY_DIR = libraries
LIBRARY_SOURCE_C = $(shell find $(LIBRARY_DIR) -name "*.c") 
LIBRARY_SOURCE_CPP = $(shell find $(LIBRARY_DIR) -name "*.cpp")
# sensor libraries
LIBRARY_INCLUDE =  -Ilibraries/Adafruit_BusIO -Ilibraries/Adafruit_ICM20X -Ilibraries/Adafruit_LIS3MDL -Ilibraries/Adafruit_LSM6DS -Ilibraries/Adafruit_Sensor
LIBRARY_INCLUDE += -Ilibraries/FreqMeasureMulti -Ilibraries/VL53L4CD -Ilibraries/FastLED/src
# communication libraries
LIBRARY_INCLUDE += -Ilibraries/FlexCAN_T4  -Ilibraries/SPI -Ilibraries/Wire -Ilibraries/QNEthernet/src 
# utility libraries
LIBRARY_INCLUDE += -Ilibraries/unity  -Ilibraries/SD -Ilibraries/SdFat/src -Ilibraries/TeensyDebug/src
# name of the output lib file
LIBRARY_LIB_NAME = liblibs.a
# lib file name stripped of initial 'lib' and '.a'
LIBRARY_LIB = libs

# Project files
PROJECT_DIR = .
PROJECT_SRC_DIR = src
PROJECT_SOURCE = $(shell find $(PROJECT_SRC_DIR) -name "*.cpp") $(shell find $(PROJECT_SRC_DIR) -name "*.c")
PROJECT_INCLUDE = -Isrc
# application filename will end up as PROJECT_NAME.hex once built
PROJECT_NAME = firmware

# Teensy41 compiler flags
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_CUSTOM -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813
# CPU flags to tailor the code for the Teensy processor
CPU_FLAGS = -DF_CPU=600000000 -D__IMXRT1062__ -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

# Base compiler flags for both C++ and C
# -O0 is not supported
COMPILE_FLAGS = -Wall -g -O2 $(CPU_FLAGS) $(TEENSY4_FLAGS) -I$(TEENSY_INCLUDE) -ffunction-sections -fdata-sections
# C++ specific flags for compiling
CPP_FLAGS = -std=gnu++17 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing
# c++ moment
CPP_FLAGS += -Wno-trigraphs -Wno-comment

# Required linker config for teensy related things
LINKING_FLAGS = -Wl,--gc-sections,--relax $(CPU_FLAGS) -Tteensy4/imxrt1062_t41.ld

# Required base libs for teensy
BASE_LIBS = -larm_cortexM7lfsp_math -lm -lstdc++

# Detects OS and adds respective paths
# Darwin is Mac, Linux is Linux
ifeq ($(UNAME),Darwin)
 ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
 $(info We've detected you are using a Mac! Consult God if this breaks.)
endif
ifeq ($(UNAME),Linux)
 ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
 $(info We've detected you're on Linux! Nerd.)
endif

# Complete compilers
COMPILER_CPP := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-g++
COMPILER_C := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-gcc
OBJCOPY := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-objcopy
GDB := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-gdb
SIZE := $(ARDUINO_PATH)/packages/teensy/tools/teensy-tools/1.59.0/teensy_size

GIT_SCRAPER = ./tools/git_scraper.cpp

# targets are phony to force it to rebuild every time
.PHONY: build build_libs clean clean_all clean_libs clean_objs clean_bins upload gdb git_scraper monitor kill restart
.DEFAULT_GOAL = build_all

# # # Main Targets # # #

# builds source, links with libraries, and constructs the .elf and .hex to be uploaded
build: clean git_scraper
	@echo [Building Source]
	@$(COMPILER_CPP) $(COMPILE_FLAGS) $(CPP_FLAGS) $(PROJECT_SOURCE) $(PROJECT_INCLUDE) $(LIBRARY_LIB_NAME) $(TEENSY_LIB_NAME) $(LIBRARY_INCLUDE) $(TEENSY_INCLUDE) $(LINKING_FLAGS) -o $(PROJECT_NAME).elf
	@$(SIZE) $(PROJECT_NAME).elf
	@echo [Constructing $(PROJECT_NAME).hex]
	@$(OBJCOPY) -O ihex -R .eeprom $(PROJECT_NAME).elf $(PROJECT_NAME).hex
	@chmod +x $(PROJECT_NAME).hex
	@echo [Cleaning Up]
	@git config --local core.hooksPath .githooks

# builds the libraries and the source
build_all: clean_all build_libs build

# builds hex, uploades it, and starts monitoring output
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run make upload
	@tycmd upload $(PROJECT_NAME).hex
# Teensy serial isn't immediately available after upload, so we wait a bit
# The teensy waits for 20 + 280 + 20 ms after power up/boot
	@sleep 0.4s
	@bash tools/monitor.sh

# # # Clean Targets # # #

# cleans up all files generated by the build process
# excludes the libraries
clean: clean_objs clean_bins

# cleans up all files generated by the build process
# includes the libraries
clean_all: clean clean_libs

# cleans up the libraries
clean_libs: 
	@rm -f *.a

# cleans up the object files
clean_objs:
	@rm -f *.o

# cleans up the binary files
clean_bins:
	@rm -f *.elf
	@rm -f *.hex

# # # Library Targets # # #

# builds the libraries
build_libs: clean_libs lib_teensy lib_libs

# builds the teensy core library
lib_teensy:
	@echo [Building Teensy Core CPP]
	@$(COMPILER_CPP) $(COMPILE_FLAGS) $(CPP_FLAGS) -c $(TEENSY_DIR)/*.cpp $(TEENSY_INCLUDE)
	@echo [Building Teensy Core C]
	@$(COMPILER_C) $(COMPILE_FLAGS) -c $(TEENSY_DIR)/*.c $(TEENSY_INCLUDE)
	@echo [Assembling Static Library]
	@ar rcs $(TEENSY_LIB_NAME) *.o
	@echo [$(TEENSY_LIB_NAME) Created in $(PROJECT_DIR)]
	@rm *.o -f
	@echo [Cleaning Up]

# builds the external libraries
lib_libs: 
	@echo [Building Libraries C]
	@$(COMPILER_C) $(COMPILE_FLAGS) -c $(LIBRARY_SOURCE_C) $(LIBRARY_INCLUDE) $(TEENSY_INCLUDE)
	@echo [Building Libraries CPP]
	@$(COMPILER_CPP) $(COMPILE_FLAGS) $(CPP_FLAGS) -c $(LIBRARY_SOURCE_CPP) $(LIBRARY_INCLUDE) $(TEENSY_INCLUDE) 
	@echo [Assembling Static Library]
	@ar rcs $(LIBRARY_LIB_NAME) *.o
	@echo [$(LIBRARY_LIB_NAME) Created in $(PROJECT_DIR)]
	@rm *.o -f
	@echo [Cleaning Up]

# # # Utility Targets # # #

help: 
	@echo "Basic usage: make [target]"
	@echo "Targets:"
	@echo "  build:        compiles the source code and links with libraries"
	@echo "  build_all:    builds the libraries and the source"
	@echo "  upload:       builds the source and uploads it to the Teensy"
	@echo "  gdb:          starts GDB and attaches to the firmware running on a connected Teensy"
	@echo "  build_libs:   builds the external libraries"
	@echo "  monitor:      monitors any actively running firmware and displays serial output"
	@echo "  kill:         stops any running firmware"
	@echo "  restart:      restarts any running firmware"

# starts GDB and attaches to the firmware running on a connected Teensy
# calls a script to prepare the GDB environment, this finds the exact port Teensy is connected to
gdb:
	@echo [Starting GDB]
	@bash tools/prepare_gdb.sh
	@$(GDB) -x ./tools/gdb_commands.txt --args $(PROJECT_NAME).elf

# compiles, runs, and cleans up the git_scraper tool which stores the current git info in a header file
git_scraper:
	@g++ $(GIT_SCRAPER) -o ./tools/git_scraper
	@./tools/git_scraper
	@rm ./tools/git_scraper

# monitors currently running firmware on robot
monitor:
	@echo [Monitoring]
	@bash tools/monitor.sh

# resets teensy and switches it into boot-loader mode, effectively stopping any execution
# this only works if power is consistent, will restart loaded firmware if turned off and on again
kill:
	@echo [Attempting to Kill Teensy]
	@tycmd reset -b

# restarts teensy
restart:
	@echo [Attempting to Restart Firmware]
	@tycmd reset
