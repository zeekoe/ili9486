cmake_minimum_required(VERSION 3.21)
project(ili9486)

set(CMAKE_CXX_STANDARD 14)

add_executable(ili9486 main.cpp)

target_link_libraries(ili9486 bcm_host)