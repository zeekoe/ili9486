cmake_minimum_required(VERSION 3.18)
project(ili9486)

set(CMAKE_C_STANDARD 99)

file(GLOB sourceFiles *.c)

add_executable(ili9486 main.c ili9486.c)

target_link_libraries(ili9486 bcm_host)