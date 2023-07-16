#!/bin/bash

CUR_DIR=$(pwd)
echo $CUR_DIR

EPOXY_DIR=EpoxyDuino

if [ -d "$EPOXY_DIR" ];
then
    echo "$EPOXY_DIR directory exists."
else
	git clone --branch v1.5.0 https://github.com/bxparks/EpoxyDuino.git > /dev/null 2>&1
	mv EpoxyDuino/EpoxyDuino.mk EpoxyDuino/EpoxyDuino.mk.bak
	# Use custom makefile
	# TODO: add a parameter to omit libraries
	cp AmbovisEpoxyDuino.mk EpoxyDuino/EpoxyDuino.mk
    cd EpoxyDuino/cores/epoxy
    touch wiring_private.h
    cp pins_arduino_avr.h pins_arduino.h
    sed -i.bkp '65i \
 \
#ifndef LSBFIRST \
#define LSBFIRST 0 \
#endif \
#ifndef MSBFIRST\
#define MSBFIRST 1\
#endif\
typedef enum _BitOrder {\
	SPI_BITORDER_MSBFIRST = MSBFIRST,\
	SPI_BITORDER_LSBFIRST = LSBFIRST,\
} BitOrder;\
' SPI.h

    cd ../../..
fi

if [ ! -d "$CUR_DIR/AUnit" ];
then
	echo "AUnit not installed. Installing..."
	git clone https://github.com/bxparks/AUnit.git --branch v1.7.0 > /dev/null 2>&1
else
	echo "AUnit already installed."
fi

if [ ! -d "$CUR_DIR/LiquidCrystal" ];
then
	echo "LiquidCrystal not installed. Installing..."
	git clone --branch 1.0.7 https://github.com/arduino-libraries/LiquidCrystal.git > /dev/null 2>&1
else
	echo "LiquidCrystal already installed."
fi

if [ ! -d "$CUR_DIR/Adafruit_BusIO" ];
then
	git clone --branch 1.7.3 https://github.com/adafruit/Adafruit_BusIO.git > /dev/null 2>&1
	cd Adafruit_BusIO/
	sed -i.bak -e '31,32d' Adafruit_SPIDevice.h
	cd ../
fi

git clone --branch 2.3.0 https://github.com/adafruit/Adafruit_ADS1X15.git > /dev/null 2>&1
git clone --branch 1.10.9 https://github.com/adafruit/Adafruit-GFX-Library.git > /dev/null 2>&1
git clone --branch 1.5.6 https://github.com/adafruit/Adafruit_ILI9341.git > /dev/null 2>&1
        

