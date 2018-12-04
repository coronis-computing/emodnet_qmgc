cmake_minimum_required(VERSION 2.8)
project("EMODnet Quantized Mesh Generator for Cesium")

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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

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

# Cesium-terrain-builder library
find_package(CTB)
if(NOT CTB_FOUND)
    message(FATAL_ERROR "Cesium-Terrain-Builder not found, required!")
endif()
include_directories(${CTB_INCLUDE_DIR})

# GDAL library
find_package(GDAL)
if(NOT GDAL_FOUND)
    message(FATAL_ERROR "The GDAL library cannot be found on the system")
endif()
include_directories(${GDAL_INCLUDE_DIRS})

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
