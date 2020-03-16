cmake_minimum_required(VERSION 2.8)
project(uartcpp)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)


add_executable( uartcpp ${sourcefiles} uart.cpp)




