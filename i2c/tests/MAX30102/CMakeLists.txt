cmake_minimum_required(VERSION 3.0.0)
project(MAX30102 VERSION 0.1.0)

include_directories(include)

link_libraries()

add_executable(demo main.c src/MAX30102.c)