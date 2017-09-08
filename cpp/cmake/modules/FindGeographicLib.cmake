# Find the GeographicLib library
#
# Variables set:
#    GEOGRAPHICLIB_FOUND = TRUE
#    GeographicLib_INCLUDE_DIRS
#    GeographicLib_LIBRARIES
#    GeographicLib_LIBRARY_DIRS
#
# Author: Ricard Campos (rcampos@eia.udg.edu)

find_library (  GeographicLib_LIBRARIES 
		NAMES Geographic
  		PATHS 	/usr/local 
			/usr/local/lib
			/opt/local/lib
			/sw/lib )

if (GeographicLib_LIBRARIES)
  get_filename_component( GeographicLib_LIBRARY_DIRS "${GeographicLib_LIBRARIES}" PATH )
endif ()

find_path( GeographicLib_INCLUDE_DIRS 
	   NAMES GeographicLib/Config.h
	   PATHS /usr/include
  		 /opt/local/include
  		 /usr/local/include
  		 /sw/include )

# (From CMake doc) If the variables <var1> to <varN> are all valid, then <UPPERCASED_NAME>_FOUND will be set to TRUE. If DEFAULT_MSG is given as second argument, then the function will generate itself useful success and error messages. 
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args( GeographicLib 
				   DEFAULT_MSG
  				   GeographicLib_LIBRARY_DIRS
				   GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS )

# (From CMake doc) Mark the named cached variables as advanced. An advanced variable will not be displayed in any of the cmake GUIs unless the show advanced option is on. 
mark_as_advanced( GeographicLib_LIBRARY_DIRS 
		  GeographicLib_LIBRARIES
		  GeographicLib_INCLUDE_DIRS )
