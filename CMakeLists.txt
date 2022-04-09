# @copyright Copyright (c) 2022 Creator, Inc. MIT license.

cmake_policy(SET CMP0048 NEW)
set(CMAKE_PROJECT_VERSION 1.0.0)
project(creator-motor-drivers VERSION ${CMAKE_PROJECT_VERSION})

# CMake version tested from 3.12 through 3.23
cmake_minimum_required(VERSION 3.12)

# prefer static linking to shared objects
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_FIND_LIBRARY_SUFFIXES .a .so)

# c/c++ standard
# this project is tested compatible with C++11 through C++20
set(CMAKE_CXX_STANDARD 11)
add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:-std=c11>
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++${CMAKE_CXX_STANDARD}>
  -fPIC)

# boost
# this project is tested compatible with boost 1.65 through 1.71
set(Boost_USE_STATIC_LIBS ON)
find_package(
    Boost REQUIRED
    COMPONENTS atomic
               chrono
               date_time
               system
               thread
               timer)

# compile options
add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-Weffc++>
  -Wall
  -Werror)

# shared include directories
include_directories("${PROJECT_SOURCE_DIR}")

# add subdirectories to be included in the build
add_subdirectory(libs)
add_subdirectory(drivers)
