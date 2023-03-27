#!/bin/bash

CUR_DIR=$(pwd)
echo $CUR_DIR

EPOXY_DIR=EpoxyDuino2
ARDUINO_DIR=$1

if [ ! -d "$ARDUINO_DIR" ];
then
    echo "$ARDUINO_DIR directory does not exist."
    exit 1;
fi

if [ -d "$EPOXY_DIR" ];
then
    echo "$EPOXY_DIR directory exists."
else
	# git clone --branch v1.5.0 https://github.com/bxparks/EpoxyDuino.git > /dev/null 2>&1
	# Some local adjustments were made on the downloaded EpoxyDuino project.
	# beware if this is downloaded again, the tests won't work. 
fi

ARDUINO_LIBS="$ARDUINO_DIR/libraries"
# echo "Moving required dependencies to $ARDUINO_LIBS"
# cd $ARDUINO_LIBS

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
	git clone https://github.com/arduino-libraries/LiquidCrystal.git > /dev/null 2>&1
else
	echo "LiquidCrystal already installed."
fi

git clone --branch 2.3.0 https://github.com/adafruit/Adafruit_ADS1X15.git > /dev/null 2>&1
git clone --branch 1.7.3 https://github.com/adafruit/Adafruit_BusIO.git > /dev/null 2>&1
git clone --branch 1.10.9 https://github.com/adafruit/Adafruit-GFX-Library.git > /dev/null 2>&1
git clone --branch 1.5.6 https://github.com/adafruit/Adafruit_ILI9341.git > /dev/null 2>&1
        
cd Adafruit_BusIO/
sed -i.bak -e '31,32d' Adafruit_SPIDevice.h
cd ../

if [ ! -d "$ARDUINO_LIBS/AMBTestable/src" ];
then
	mkdir -p "$ARDUINO_LIBS/AMBTestable/src"
else
	echo "Clean old links"
	cd "$ARDUINO_LIBS/AMBTestable/src"
	DIR_TO_DEL="$ARDUINO_LIBS/AMBTestable/src"
	for f in "$DIR_TO_DEL"/*.o
	do
  	rm "${f}"
	done
	find . -type l -exec rm {} \;
fi

## create links to the current compilable files
ln -sf "$CUR_DIR/ambovis/pinout.h" "$ARDUINO_LIBS/AMBTestable/src/pinout.h"
ln -sf "$CUR_DIR/ambovis/defaults.h" "$ARDUINO_LIBS/AMBTestable/src/defaults.h"
ln -sf "$CUR_DIR/ambovis/display.h" "$ARDUINO_LIBS/AMBTestable/src/display.h"
ln -sf "$CUR_DIR/ambovis/display.cpp" "$ARDUINO_LIBS/AMBTestable/src/display.cpp"
ln -sf "$CUR_DIR/ambovis/MechanicalVentilation.h" "$ARDUINO_LIBS/AMBTestable/src/MechanicalVentilation.h"
ln -sf "$CUR_DIR/ambovis/MechanicalVentilation.cpp" "$ARDUINO_LIBS/AMBTestable/src/MechanicalVentilation.cpp"
ln -sf "$CUR_DIR/ambovis/menu.h" "$ARDUINO_LIBS/AMBTestable/src/menu.h"
ln -sf "$CUR_DIR/ambovis/menu.cpp" "$ARDUINO_LIBS/AMBTestable/src/menu.cpp"
ln -sf "$CUR_DIR/ambovis/alarms.h" "$ARDUINO_LIBS/AMBTestable/src/alarms.h"
ln -sf "$CUR_DIR/ambovis/alarms.cpp" "$ARDUINO_LIBS/AMBTestable/src/alarms.cpp"
ln -sf "$CUR_DIR/ambovis/initialactions.h" "$ARDUINO_LIBS/AMBTestable/src/initialactions.h"
ln -sf "$CUR_DIR/ambovis/initialactions.cpp" "$ARDUINO_LIBS/AMBTestable/src/initialactions.cpp"
ln -sf "$CUR_DIR/ambovis/sensorcalculation.h" "$ARDUINO_LIBS/AMBTestable/src/sensorcalculation.h"
ln -sf "$CUR_DIR/ambovis/sensorcalculation.cpp" "$ARDUINO_LIBS/AMBTestable/src/sensorcalculation.cpp"
