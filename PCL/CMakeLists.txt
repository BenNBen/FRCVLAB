#Purpose: used to build executable of filter.cpp file 
#Author: Ben Barriage
#Date: 10/5/16
# c 2016 BBarriage
# c 2016 Fordham University
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(filter)

find_package(PCL 1.2 REQUIRED)

SET(GCC_COVERAGE_COMPILE_FLAGS "-g")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

add_executable(filter filter.cpp)

target_link_libraries (filter ${PCL_LIBRARIES})
