# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vestal-vamp)

target_sources(app PRIVATE src/main.c src/display.c src/adc.c src/dac.c src/timer.c)
