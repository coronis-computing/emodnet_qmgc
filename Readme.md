# EMODnet Quantized Mesh Generator for Cesium

Library for generating terrain tiles in Cesium's quantized-mesh format from a GDAL raster file.

**Documentation:** https://coronis-computing.github.io/emodnet_qmgc/ (user guide, tutorial and [API reference](https://coronis-computing.github.io/emodnet_qmgc/api/)).

## Requirements

This project depends on the following C++ libraries:

* Boost (program_options): http://www.boost.org/
* ZLib: https://www.zlib.net/
* GDAL: http://www.gdal.org/ (version 3.0 or later)
* GeographicLib: https://geographiclib.sourceforge.io/
* CGAL: https://www.cgal.org/ (version 5.x, tested with 5.6)
* meshoptimizer: https://github.com/zeux/meshoptimizer (with small modifications, source included in this project) 
* JSON for modern C++: https://github.com/nlohmann/json (source included in this project)

Python 3 is only needed to run the scripts in the `scripts` folder, which are optional: `simple_tiles_server.py`, a test server for the tiles, and `create_layer_file.py`, described below.

### Ubuntu 24.04

All the required libraries are available in the official Ubuntu 24.04 repositories (which provide GDAL 3.8 and CGAL 5.6). Install them with:

```
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    zlib1g-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libcgal-dev \
    libgdal-dev \
    libgeographiclib-dev \
    libproj-dev \
    gdal-bin \
    python3
```

Optionally, install `doxygen` to be able to build the documentation with `make doc`.

After that, the project can be compiled directly with the commands listed above. To install the tools system-wide, run `sudo make install` from the `build` directory.

## Compilation

This project uses the typical cmake pipeline, and requires a C++14 compiler. Thus, assuming that all the libraries listed above are installed, it should be as easy as:

```
mkdir build
cd build
cmake ..
make
```

The developer tools in `src/dev-tests` are also built by default. To skip them, which shortens the build, use `cmake .. -DQMGC_BUILD_DEV_TESTS=OFF`.

## Documentation

The full documentation, including a user guide and the API reference, is published on the [web of this project](https://coronis-computing.github.io/emodnet_qmgc/). It is rebuilt and published automatically by GitHub Actions (`.github/workflows/docs.yml`) on every push to `master`.

It has two parts, which can also be built locally:

* **User guide** (sources in `docs/` and `mkdocs.yml`), built with [MkDocs](https://www.mkdocs.org/) and [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/). From the root of the repository:

    ```
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements-docs.txt
    mkdocs serve                # live preview at http://127.0.0.1:8000
    mkdocs build --strict       # static site in site/
    ```

* **API reference**, built with Doxygen from the comments in the source code. Install `doxygen` and, from the `build` folder, run:

    ```
    make doc
    ```

    The result is written to `build/docs/html/index.html`.

See the [documentation guide](https://coronis-computing.github.io/emodnet_qmgc/developer/documentation/) for more details.

## Usage

The main functionality of the project is provided by the `qm_tiler` app. A simple usage is:

```
qm_tiler -i <input_raster> -o <output_tiles_dir>
```

However, the method has many parameters that may be tuned, and that are explained in the [user guide](https://coronis-computing.github.io/emodnet_qmgc/user-guide/qm_tiler/). You can list them by running:

```
qm_tiler -h
```

The input can be any raster supported by GDAL, in any spatial reference system (e.g., a GeoTIFF, an ESRI ASCII grid, or a VRT joining several of them created with `gdalbuildvrt`).

Besides the TMS pyramid of tiles, `qm_tiler` writes the `layer.json` file required by Cesium in the output folder. If the folder already has one, the zooms it lists are kept, so a large pyramid can be built in several runs (e.g., `-s 12 -e 6`, then `-s 5 -e 0`). For an already existing tileset without a `layer.json` file (e.g., one created with an older version), `scripts/create_layer_file.py <tiles_dir>` creates it from the tiles found in the folder.

The tiles include per-vertex normals (Cesium's `octvertexnormals` extension), which Cesium needs in order to shade the terrain. The `layer.json` file declares them, but Cesium only uses them when they are requested on the client side, and lighting is enabled:

```
viewer.terrainProvider = await Cesium.CesiumTerrainProvider.fromUrl(url, { requestVertexNormals: true });
viewer.scene.globe.enableLighting = true;
```

If the normals are not needed, they can be left out with `qm_tiler --no-normals` (and `create_layer_file.py --no-normals`).

Finally, in order to easily test the project, we also provide `scripts/simple_tiles_server.py <output_tiles_dir>`, a simple web server for the tiles that sets the headers Cesium needs (CORS, and gzip encoding for the `.terrain` files).

Please refer to the [documentation](https://coronis-computing.github.io/emodnet_qmgc/) of this project for further information, including a step-by-step [tutorial](https://coronis-computing.github.io/emodnet_qmgc/getting-started/tutorial/).

## Known issues

* With the `delaunay` TIN creation strategy, the borders of neighboring tiles are not shared, so each tile computes the normals of its border vertices on its own. This may show as faint lighting seams along the tile borders. The other strategies reuse the normals of the neighboring tiles, so the shading is continuous.

* Creating tiles at low zoom levels from very large DEMs requires resampling large parts of the source raster for each tile. The raster is warped through GDAL, which automatically uses the overviews of the source dataset when available, so it is recommended to build them beforehand (e.g., `gdaladdo -r average <input_raster>`).

## Acknowledgements

This project has been developed by Coronis Computing S.L. within the EMODnet Bathymetry project.

* EMODnet: http://www.emodnet.eu/
* EMODnet (bathymetry): http://www.emodnet-bathymetry.eu/
* Coronis: http://www.coronis.es
