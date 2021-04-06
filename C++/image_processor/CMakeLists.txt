cmake_minimum_required(VERSION 3.13)

project(image_processor)

set(CMAKE_CXX_STANDARD 17 )
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wno-multichar")

set(SOURCE_FILES  src/bitmap.c  src/improc.cpp)

include_directories(include)


add_executable(image_processor ${SOURCE_FILES} main.cpp  )
