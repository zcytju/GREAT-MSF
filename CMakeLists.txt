# Minimum requirement of CMake version : 3.20.0
cmake_minimum_required(VERSION 3.20)
# Project name and version number
project(GREAT-MSF VERSION 0.1.0)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin/)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib/)

add_subdirectory(src)
