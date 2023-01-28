#!/bin/bash

CUR_DIR=$(pwd)
echo $CUR_DIR

EPOXY_DIR=EpoxyDuino
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
	git clone https://github.com/bxparks/EpoxyDuino.git > /dev/null 2>&1
	# Some local adjustments were made on the downloaded EpoxyDuino project.
	# beware if this is downloaded again, the tests won't work. 
fi

ARDUINO_LIBS="$ARDUINO_DIR/libraries"
echo "Moving required dependencies to $ARDUINO_LIBS"
if [ ! -d "$ARDUINO_LIBS/EpoxyDuino" ];
	mv $EPOXY_DIR $ARDUINO_LIBS
fi

cd $ARDUINO_LIBS

if [ ! -d "$ARDUINO_LIBS/AUnit" ];
then
	echo "AUnit not installed"
else
	mkdir to_delete
	cd to_delete
	git clone https://github.com/bxparks/AUnit.git --branch v1.7.0 > /dev/null 2>&1
fi

if [ ! -d "$ARDUINO_LIBS/AMBTestable/src" ];
then
	mkdir -p "$ARDUINO_LIBS/AMBTestable/src"
else
	rm -R "$ARDUINO_LIBS/AMBTestable/src/*"
fi

#create links to the current compilable files
ln -sf "$CUR_DIR/ambovis/pinout.h" "$ARDUINO_LIBS/AMBTestable/src/pinout.h"
ln -sf "$CUR_DIR/ambovis/defaults.h" "$ARDUINO_LIBS/AMBTestable/src/defaults.h"
ln -sf "$CUR_DIR/ambovis/display.h" "$ARDUINO_LIBS/AMBTestable/src/display.h"
ln -sf "$CUR_DIR/ambovis/display.cpp" "$ARDUINO_LIBS/AMBTestable/src/display.cpp"

