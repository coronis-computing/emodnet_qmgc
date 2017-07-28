cmake_minimum_required(VERSION 2.6)
project("EMODnet tools")

# C++11 related
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#
#if(NOT MSVC)
#  if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#  elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#  else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#  endif()
#endif()

# Boost library
find_package(Boost COMPONENTS program_options
                              filesystem
                              system
                              iostreams
                              REQUIRED)

# We need zlib
find_package(ZLIB)
if(NOT ZLIB_FOUND)
    message(FATAL_ERROR "The zlib library cannot be found on the system")
endif()
include_directories(${ZLIB_INCLUDE_DIRS})

# Build srcs
include_directories("./src")
include_directories("./apps")
add_subdirectory("apps")

