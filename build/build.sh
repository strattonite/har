#!/bin/sh

mkdir -p work
mkdir -p work/boards/har
mkdir -p work/demos/har/src
mkdir -p work/demos/har/src/btstack

cp -r ../sdk/ksdk1.1.0/*							work
cp ../src/*.c											work/demos/har/src/
cp ../src/*.h											work/demos/har/src/
cp ../src/CMakeLists.txt								work/demos/har/armgcc/har/
cp ../src/startup_MKL03Z4.S								work/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
cp ../src/gpio_pins.c									work/boards/har
cp ../src/gpio_pins.h									work/boards/har

cd work/lib/ksdk_platform_lib/armgcc/KL03Z4 && ./clean.sh; ./build_release.sh
cd ../../../../demos/har/armgcc/har && ./clean.sh; ./build_release.sh
echo "\n\nNow, run\n\n\t/Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands\n\n"

