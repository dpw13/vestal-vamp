PROGRAM_NAME := io_expander
SRC_DIR := src
BUILD_DIR := build

PREFIX := /opt/avr-gcc-14.1.0-x64-linux/bin/avr-
CC := ${PREFIX}gcc
OBJCOPY := ${PREFIX}objcopy
OBJDUMP := ${PREFIX}objdump
SIZE := ${PREFIX}size
MCU := attiny816

# Note that we start .text at 0x200 in order to not squash the bootloader
CFLAGS := -gdwarf-2 -mmcu=${MCU} \
        -ffunction-sections -fdata-sections \
        -Wl,--discard-locals,--relax,--gc-sections,--undefined=_mmcu \
		-Wl,--section-start=.text=0x0200 -Ttext=0x0200 \
        -DF_CPU=20000000 -Wall -Os
CCFLAGS := -std=c++11 ${CFLAGS}

SOURCES := $(wildcard src/*.c)

all: ${BUILD_DIR}/${PROGRAM_NAME}.hex

# Compile and build the program for Atmega328P
${BUILD_DIR}/${PROGRAM_NAME}.elf: ${SOURCES}
	mkdir -p ${BUILD_DIR}
	$(CC) $(CFLAGS) -o $@ ${SOURCES}
	$(OBJDUMP) -P mem-usage $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -j .rodata -O ihex $^ $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -j .rodata -O binary $^ $@

# Remove build directory with all built files
clean:
	rm -rf ${BUILD_DIR} 