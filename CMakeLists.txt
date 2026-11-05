cmake_minimum_required(VERSION 3.10)
project(emodnet_qmgc
        VERSION 0.3.0
        DESCRIPTION "EMODnet Quantized Mesh Generator for Cesium"
        LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Extent the module path
# -----------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

# -----------------------------------------------------------------------------
# Default build options
# -----------------------------------------------------------------------------

# Build optimized by default: without optimization, CGAL code runs many times slower
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# The developer tools in src/dev-tests are not needed to create tiles
option(QMGC_BUILD_DEV_TESTS "Build the developer tools in src/dev-tests" ON)

# C++14 is the minimum required by CGAL 5.x
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# Required packages
# -----------------------------------------------------------------------------

# PThreads
find_package(Threads REQUIRED)

# CGAL library
find_package(CGAL)
include( ${CGAL_USE_FILE} )

# Boost library
find_package(Boost COMPONENTS program_options
                              filesystem
                              system
                              REQUIRED)
if(NOT Boost_FOUND)
    message(FATAL_ERROR "The boost library cannot be found on the system, required!")
endif()

message("Boost libs = ${Boost_LIBRARIES}")

# ZLib library
find_package(ZLIB)
if(NOT ZLIB_FOUND)
    message(FATAL_ERROR "The zlib library cannot be found on the system, required!")
endif()
include_directories(${ZLIB_INCLUDE_DIRS})

# GDAL library
find_package(GDAL)
if(NOT GDAL_FOUND)
    message(FATAL_ERROR "The GDAL library cannot be found on the system")
endif()
include_directories(${GDAL_INCLUDE_DIRS})

find_package(GeographicLib REQUIRED)
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories (${GeographicLib_INCLUDE_DIRS})
    add_definitions (${GeographicLib_DEFINITIONS})
endif ()

# Doxygen for documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.cfg ${CMAKE_CURRENT_BINARY_DIR}/doxyfile @ONLY)
    add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif(DOXYGEN_FOUND)

# Project-related include dirs
include_directories("src")
include_directories("3rdParty")
include_directories("3rdParty/json")

# -----------------------------------------------------------------------------
# Compilation
# -----------------------------------------------------------------------------
include(GNUInstallDirs)
add_subdirectory("src")
