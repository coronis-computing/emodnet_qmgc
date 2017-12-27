# Look for cesium-terrain-builder library in common dirs or env vars
# Author: Ricard Campos (rcampos@eia.udg.edu)

# Check for cmake/env-defined vars
set( CTB_INCLUDE_SEARCH_PATHS ${CTB_INCLUDE_HITS} $ENV{CTB_INCLUDE_HITS} )
set( CTB_LIBRARY_SEARCH_PATHS ${CTB_LIBRARY_HINTS} $ENV{CTB_LIBRARY_HINTS} )

# Check include in common paths
FIND_PATH(CTB_INCLUDE_DIR NAMES TerrainTile.hpp
        PATHS
        ${CTB_INCLUDE_SEARCH_PATHS}
        /usr/include/ctb
        /usr/include
        /opt/local/include
        /opt/local/include/ctb
        /usr/local/include
        /usr/local/include/ctb
        /sw/include
        /sw/include/ctb
        )

# Check library in common paths
FIND_LIBRARY(CTB_LIBRARY NAMES libctb.so
        PATHS
        ${CTB_LIBRARY_HINTS}
        /usr/lib/ctb
        /usr/lib
        /opt/local/lib
        /opt/local/lib/ctb
        /usr/local/lib
        /usr/local/lib/ctb
        /sw/lib
        /sw/lib/ctb
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ctb "Could NOT find Cesium-Terrain-Builder\nNOTE: If you don't have CTB installed in default system paths, you should set CTB_INCLUDE_HINTS and CTB_LIBRARY_HINTS variables (either in cmake or as environment vars).\n" CTB_INCLUDE_DIR CTB_LIBRARY )

