# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vestal-vamp)

# GCC doesn't like the compound literal initializers used for the settings structure.
set_source_files_properties(src/settings.c PROPERTIES COMPILE_FLAGS -Wno-missing-braces)
target_sources(app PRIVATE
        src/main.c
        src/display.c
        src/adc.c
        src/dac.c
        src/timer.c
        src/freq_buffer.c
        src/fft_dma.c
        src/ifft_dma.c
        src/settings.c
)
#zephyr_library_compile_options("-E")