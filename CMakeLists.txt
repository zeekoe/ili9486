cmake_minimum_required(VERSION 3.18)
project(ili9486)

set(CMAKE_CXX_STANDARD 14)

file(GLOB sourceFiles *.cpp)

add_executable(ili9486 main.cpp display.cpp spi.cpp)

target_link_libraries(ili9486 bcm_host)