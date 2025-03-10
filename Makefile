include setup.conf

all: har

test:
	@echo "$(CMAKE_CURRENT_SOURCE_DIR)"

har:
	mkdir -p build/work
	mkdir -p build/work/boards/har
	mkdir -p build/work/demos/har/src
	mkdir -p build/work/demos/har/armgcc/har

	cp -r sdk/ksdk1.1.0/*			build/work
	cp src/*.h							build/work/demos/har/src/
	cp src/*.c							build/work/demos/har/src/
	cp src/startup_MKL03Z4.S			build/work/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
	cp src/CMakeLists.txt				build/work/demos/har/armgcc/har/CMakeLists.txt
	cp src/gpio_pins.c				build/work/boards/har
	cp src/gpio_pins.h				build/work/boards/har
	cp src/config.h					build/work/boards/har

	cd build/work/lib/ksdk_platform_lib/armgcc/KL03Z4 && ./clean.sh; ./build_release.sh
	cd build/work/demos/har/armgcc/har && ./clean.sh; ./build_release.sh
	@echo "\n\nNow, run\n\n\tmake load-har\n\n"

load-warp:
	$(JLINKPATH) -device MKL03Z32XXX4 -if SWD -speed 10000 -CommanderScript warp.jlink.commands

clean:
	rm -rf build/work