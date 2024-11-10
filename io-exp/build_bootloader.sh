#!/bin/bash

# Build optiboot_x for the ATtiny816 feather board

export GCCROOT=/opt/avr-gcc-14.1.0-x64-linux/bin/
export TARGET=attiny816

OPTIBOOT=megaTinyCore/megaavr/bootloaders/optiboot_x

pushd $OPTIBOOT
rm -f *.hex *.elf
make -f Makefile optiboot_feather.hex UARTTX=B2 TIMEOUT=8 LED=C0 ENTRYCOND_REQUIRE=0x35
popd
mv $OPTIBOOT/optiboot_feather.hex .

# Programming:
# python prog.py -d attiny816 -u COM4 -a write -f optiboot_feather.hex
# Fuses:
# SYSCFG0 5:0xf6 (UPDI pin is UPDI)
# SYSCFG 6:0x04 (8 ms startup delay)
# APPEND 7:0x00
# BOOTEND 8:0x02
# python prog.py -d attiny816 -u COM4 --fuses 5:0xf2 6:0x04 7:0x00 8:0x02
