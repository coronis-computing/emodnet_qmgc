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

## Tools

The main functionality of the project is provided by the `qm_tiler` app:

```
Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format:
  -h [ --help ]                         Produce help message
  -i [ --input ] arg                    Input terrain file to parse
  -o [ --output-dir ] arg (=terrain_tiles_qm)
                                        The output directory for the tiles
  -s [ --start-zoom ] arg (=-1)         The zoom level to start at. This should
                                        be greater than the end zoom level 
                                        (i.e., the TMS pyramid is constructed 
                                        from bottom to top). If smaller than 
                                        zero, defaults to the maximum zoom 
                                        possible according to DEM resolution.
  -e [ --end-zoom ] arg (=0)            The zoom level to end at. This should 
                                        be less than the start zoom level 
                                        (i.e., the TMS pyramid is constructed 
                                        from bottom to top).
  -b [ --bathymetry ]                   Switch to consider the input DEM as 
                                        containing depths instead of elevations
  --no-simp                             Flag disabling simplification. The 
                                        terrain will be represented with a 
                                        regular grid extracted from the rasters
                                        (similar to the old heightmap format)
  --samples-per-tile arg (=256)         Samples to take in each dimension per 
                                        tile. While TMS tiles are supposed to 
                                        comprise 256x256 pixels/samples, using 
                                        this option we can sub-sample it to 
                                        lower resolutions. Note that a smaller 
                                        sampling provides a coarser base mesh 
                                        that will be easier to simplify.
  --simp-stop-edges-count arg (=500)    Simplification stops when the number of
                                        edges is below this value.
  --simp-weight-volume arg (=0.5)       Simplification volume weight 
                                        (Lindstrom-Turk cost function, see 
                                        original reference).
  --simp-weight-boundary arg (=0.5)     Simplification boundary weight 
                                        (Lindstrom-Turk cost function, see 
                                        original reference).
  --simp-weight-shape arg (=1e-10)      Simplification shape weight 
                                        (Lindstrom-Turk cost function, see 
                                        original reference).
  --clip-high arg (=inf)                Clip values in the DEM above this 
                                        threshold.
  --clip-low arg (=-inf)                Clip values in the DEM below this 
                                        threshold.
  --num-threads arg (=1)                Number of threads used (0=max_threads)
  --scheduler arg (=rowwise)            Scheduler type. Defines the preferred 
                                        tile processing order within a zoom. 
                                        Note that on multithreaded executions 
                                        this order may not be preserved. 
                                        OPTIONS: rowwise, columnwise, 
                                        chessboard, 4connected (see 
                                        documentation for the meaning of each)
```

After creating the TMS pyramid of tiles, use the script `create_layer_file.py` to create the `layer.json` file required by Cesium.

Finally, in order to easily test the project, we also provide the `simple_tiles_server.py`, which creates a simple web server of tiles.

Please refer to the [tutorial](https://bitbucket.org/ricardcd/emodnet/wiki/Tutorial) page in the wiki of this project for further information.

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
* Coronis: http://coronis.udg.edu/
