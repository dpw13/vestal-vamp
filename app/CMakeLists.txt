# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vestal-vamp)

# GCC doesn't like the compound literal initializers used for the settings structure.
set_source_files_properties(src/settings.c PROPERTIES COMPILE_FLAGS -Wno-missing-braces)
target_sources(app PRIVATE
        src/main.c
        src/display.c
        src/filter.c
        src/adc.c
        src/dac.c
        src/io.c
        src/timer.c
        src/freq_buffer.c
        src/fft_dma.c
        src/ifft_dma.c
        src/settings.c
        src/ui.c

        src/cordic_stm32.c
        src/dfsdm_stm32.c
        src/fmac_stm32.c
        src/opamp_stm32.c
)

zephyr_code_relocate(LIBRARY modules__cmsis-dsp LOCATION ITCM_TEXT NOKEEP)
zephyr_code_relocate(FILES src/fft_dma.c src/ifft_dma.c LOCATION ITCM_TEXT NOKEEP)

#zephyr_library_compile_options("-Wc++-compat")
#zephyr_library_compile_options("-E")