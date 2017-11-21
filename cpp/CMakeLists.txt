cmake_minimum_required(VERSION 2.6)
project("EMODnet tools")

# -----------------------------------------------------------------------------
# Extent the module path
# -----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# -----------------------------------------------------------------------------
# Default build options
# -----------------------------------------------------------------------------

# C++11 related
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(NOT MSVC)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

# -----------------------------------------------------------------------------
# Required packages
# -----------------------------------------------------------------------------

# Boost library
find_package(Boost COMPONENTS program_options
                              filesystem
                              system
                              REQUIRED)

# ZLib library
find_package(ZLIB)
if(NOT ZLIB_FOUND)
    message(FATAL_ERROR "The zlib library cannot be found on the system, required!")
endif()
include_directories(${ZLIB_INCLUDE_DIRS})

# CGAL library
find_package(CGAL)
include( ${CGAL_USE_FILE} )

# Cesium-terrain-builder library
find_package(CTB)
if(NOT CTB_FOUND)
    message(FATAL_ERROR "Cesium-Terrain-Builder not found, required!")
endif()
include_directories(${CTB_INCLUDE_DIRS})

# GDAL library
find_package(GDAL)
if(NOT GDAL_FOUND)
    message(FATAL_ERROR "The GDAL library cannot be found on the system")
endif()
include_directories(${GDAL_INCLUDE_DIRS})

find_package( GeographicLib )
if( GEOGRAPHICLIB_FOUND )
    include_directories ( ${GeographicLib_INCLUDE_DIRS} )
    link_directories( ${GeographicLib_LIBRARY_DIRS} )
endif()

# OpenMP
if(USE_OPENMP)
    find_package(OpenMP)
endif()
if(USE_OPENMP AND OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    add_definitions("-DUSE_OPENMP")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(STATUS "Disabling OpenMP support")
endif()

# Project-related include dirs
include_directories("./src")
include_directories("./apps")
include_directories("./3rdParty")

# -----------------------------------------------------------------------------
# Compilation
# -----------------------------------------------------------------------------
add_subdirectory("apps")
add_subdirectory("tests")

