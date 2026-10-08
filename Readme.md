# EMODnet Quantized Mesh Generator for Cesium

Library for generating terrain tiles in Cesium's quantized-mesh format from a GDAL raster file.

## Requirements

This project depends on the following C++ libraries:

* Boost (program_options): http://www.boost.org/
* ZLib: https://www.zlib.net/
* GDAL: http://www.gdal.org/ (version 3.0 or later)
* GeographicLib: https://geographiclib.sourceforge.io/
* CGAL: https://www.cgal.org/ (version 5.x, tested with 5.6)
* meshoptimizer: https://github.com/zeux/meshoptimizer (with small modifications, source included in this project) 
* JSON for modern C++: https://github.com/nlohmann/json (source included in this project)

Also, Python is required to run the scripts provided.

## Compilation

This project uses the typical cmake pipeline, and requires a C++14 compiler. Thus, assuming that all the libraries listed above are installed, it should be as easy as:

```
mkdir build
cd build
cmake ..
make
```

Optionally, you can create the documentation using:

```
make doc
```

An already compiled version of the documentation can be found in the [web of this project](https://coronis-computing.github.io/emodnet_qmgc/html/index.html).

## Usage

The main functionality of the project is provided by the `qm_tiler` app. A simple usage is:

```
qm_tiler -i <input_raster> -o <output_tiles_dir>
```

However, the method has many parameters that may be tuned, and that are explained in the [wiki](https://github.com/coronis-computing/emodnet_qmgc/wiki). You can list them by running:

```
qm_tiler -h
```

After creating the TMS pyramid of tiles, use the script `create_layer_file.py` to create the `layer.json` file required by Cesium.

Finally, in order to easily test the project, we also provide the `simple_tiles_server.py`, which creates a simple web server of tiles.

Please refer to the [wiki](https://github.com/coronis-computing/emodnet_qmgc/wiki) page of this project for further information.

## Known issues

* Creating tiles at low zoom levels from very large DEMs requires resampling large parts of the source raster for each tile. The raster is warped through GDAL, which automatically uses the overviews of the source dataset when available, so it is recommended to build them beforehand (e.g., `gdaladdo -r average <input_raster>`).

* Alternatively, the provided `qm_tiler_large_files.py` python script builds the low zoom levels from intermediate GeoTIFF tiles of a deeper zoom level. Note that this script requires Python 2, and still relies on the `ctb-tile` tool of Cesium Terrain Builder (https://github.com/geo-data/cesium-terrain-builder), which is no longer a dependency of this project and is therefore not installed in the provided Docker image.

## Acknowledgements

This project has been developed by Coronis Computing S.L. within the EMODnet Bathymetry project.

* EMODnet: http://www.emodnet.eu/
* EMODnet (bathymetry): http://www.emodnet-bathymetry.eu/
* Coronis: http://www.coronis.es
