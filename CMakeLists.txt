# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Make sure we're not in sysbuild mode for this simple project
set(BOARD nrf52832dk_nrf52832)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nfc_charger)

# Add source files
target_sources(app PRIVATE
    src/main.c
    src/st25r3916b_driver.c
    src/opa323_adc_control.c
)

# Add include directories
target_include_directories(app PRIVATE
    src/
)