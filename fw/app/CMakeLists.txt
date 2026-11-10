# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tram_odo_fw C)

set(ODO_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../lib/odo)
file(GLOB ODO_SRC ${ODO_LIB}/src/*.c)

target_sources(app PRIVATE src/main.c ${ODO_SRC})
target_include_directories(app PRIVATE ${ODO_LIB}/include)
target_compile_options(app PRIVATE -Wall -Wextra -Werror)
