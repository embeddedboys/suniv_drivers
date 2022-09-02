cmake_minimum_required(VERSION 3.10)

project(at24cxx VERSION 0.1)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

option(CROSS_COMP "Is need cross compile project?" false)

if(${CROSS_COMP})
    message("Cross compile mode")
    set(CMAKE_C_COMPILER /home/book/100ask_imx6ull-sdk/Buildroot_2019.02/output/host/bin/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER /home/book/100ask_imx6ull-sdk/Buildroot_2019.02/output/host/bin/arm-linux-gnueabihf-g++)
    
endif(${CROSS_COMP})

include(cross_compile.cmake)
include_directories(GLOB "include")

file(GLOB SRCS "src/*.c" "src/*.cpp" main.cpp)

add_executable(demo ${SRCS})