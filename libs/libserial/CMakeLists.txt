CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

#
# Project version numbering using semantic versioning. See:
# https://semver.org/
#
# These are used to set VERSION and SOVERSION properties of the LibSerial
# libarary. See:
# - https://cmake.org/cmake/help/latest/prop_tgt/SOVERSION.html
# - https://cmake.org/cmake/help/latest/prop_tgt/VERSION.html
#
PROJECT(LibSerial LANGUAGES C CXX VERSION 1.0.0)

#
# Project specific options and variables
#
OPTION(INSTALL_STATIC "Install static library." ON)
OPTION(INSTALL_SHARED "Install shared object library." ON)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Create compile_commands.json file so that it may be used by various
# editors/plugins/IDEs that support it.
#
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)

#
# Use GNU standard installation directories. CMake will use /usr/local
# as the default install directory. Users may override this by setting
# CMAKE_INSTALL_PREFIX. For example:
#
# cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr ..
#
INCLUDE(GNUInstallDirs)

#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to FIND_PACKAGE(Threads).
#
SET(THREADS_HAVE_PTHREAD_ARG 1)
FIND_PACKAGE(Boost COMPONENTS unit_test_framework REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

#
# Use -DCMAKE_BUILD_TYPE=Release or -DCMAKE_BUILD_TYPE=Debug to let CMake
# decide whether to use debug or optimization flags. We should not hard-code
# them here. Similarly, let CMake handle flags needed for shared object files
# (such as -fPIC). Additionally, "-pthread" flag will also be handled by CMake
# via the use of CMAKE_THREAD_LIBS_INIT (cmake < 3.1) or Threads::Threads.
#
ADD_DEFINITIONS(
  -Wall
  -Wcast-align
  -Wchar-subscripts
  -Wdouble-promotion
  -Wextra
  -Wfatal-errors
  -Wformat
  -Wformat-security
  -Wlogical-op
  -Wno-format-extra-args
  -Wno-long-long
  -Wno-parentheses
  -Wno-psabi
  -Wno-variadic-macros
  -Woverlength-strings
  -Wpacked
  -Wpointer-arith
  -Wunused-local-typedefs
  -Wwrite-strings
  -Wno-noexcept-type
  -fstrict-aliasing
  -fno-check-new
  -fno-common
  -fvisibility=default
  -pedantic
  )

#
# We require support for C++14 features in the compiler. This can be
# achieved using CMAKE_CXX_STANDARD in CMake version 3.1 or later. For
# older versions we have to add the compiler flags manually.
#
macro(USE_CXX14)
    IF (CMAKE_VERSION VERSION_LESS "3.1")
        IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
        ENDIF ()
    ELSE ()
        SET(CMAKE_CXX_STANDARD 14)
        SET(CMAKE_STANDARD_REQUIRES ON)
    ENDIF ()
endmacro(USE_CXX14)
USE_CXX14()

SET(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
SET(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")

INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIRS}
  )

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_INSTALL_MESSAGE ALWAYS)

ADD_SUBDIRECTORY(src)
