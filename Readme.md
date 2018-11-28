# EMODnet Quantized Mesh Generator for Cesium

Library for generating terrain tiles in Cesium's quantized-mesh format from a GDAL raster file.

## Requirements

This project depends on the following C++ libraries:

* Boost (program_options): http://www.boost.org/
* ZLib: https://www.zlib.net/
* Cesium Terrain Builder (libctb): https://github.com/geo-data/cesium-terrain-builder
* GDAL: http://www.gdal.org/
* CGAL: https://www.cgal.org/
* meshoptimizer: https://github.com/zeux/meshoptimizer (with small modifications, source included in this project) 
* JSON for modern C++: https://github.com/nlohmann/json (source included in this project)

Also, Python is required to run the scripts provided.

## Compilation

This project uses the typical cmake pipeline. Thus, assuming that all the libraries listed above are installed, it should be as easy as:

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

However, the method has many parameters that may be tuned, and that are explained in the [tutorial](https://github.com/coronis-computing/emodnet_qmgc/wiki). You can list them by running:

```
qm_tiler -h
```

After creating the TMS pyramid of tiles, use the script `create_layer_file.py` to create the `layer.json` file required by Cesium.

Finally, in order to easily test the project, we also provide the `simple_tiles_server.py`, which creates a simple web server of tiles.

Please refer to the [tutorial](https://github.com/coronis-computing/emodnet_qmgc/wiki) page in the wiki of this project for further information.

## Known issues

* As previously stated, this project depends on Cesium Terrain Builder library (https://github.com/geo-data/cesium-terrain-builder), and thus it shares the same limitation when creating tiles at lower zoom levels from large DEMs. In this case, an error similar to the following one may raise:

```
ERROR 1: Integer overflow : nSrcXSize=20979, nSrcYSize=28800
ERROR 1: IReadBlock failed at X offset 0, Y offset 0
terminate called after throwing an instance of 'ctb::CTBException'
```
    
* If the case above raises, you can use the provided `qm_tiler_large_images.py` python script, which basically automates the solution proposed in Cesium Terrain Builder's documentation (https://github.com/geo-data/cesium-terrain-builder). 

## Acknowledgements

This project has been developed by Coronis Computing S.L. within the EMODnet Bathymetry project.

* EMODnet: http://www.emodnet.eu/
* EMODnet (bathymetry): http://www.emodnet-bathymetry.eu/
* Coronis: http://www.coronis.es
