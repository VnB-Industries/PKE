cmake_minimum_required(VERSION 3.20.0)

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/../common/keys.h)
  message(FATAL_ERROR "common/keys.h is missing: run scripts/gen_keys.py first")
endif()

# Flash through the XIAO's UF2 bootloader (double-tap reset first) instead of a J-Link.
# The board file expects the Sense bootloader ID; the XIAO nRF52840 Plus reports this one.
set(BOARD_FLASH_RUNNER uf2)
macro(app_set_runner_args)
  board_runner_args(uf2 "--board-id=nRF52840-SeeedXiao-v1")
endmacro()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pke_car)

target_sources(app PRIVATE
  src/main.c
  src/rpa.c
  src/auth_client.c
  src/ranging.c
  src/vehicle.c
  src/actions.c
  src/door.c
  src/tap_listener.cpp
  src/tap/detector.cpp
  src/tap/pattern.cpp
  src/tap/tap_features.cpp
  ../common/pke_crypto.c
)
target_include_directories(app PRIVATE ../common src/tap)
