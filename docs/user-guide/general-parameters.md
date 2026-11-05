# General parameters

These parameters do not depend on the TIN creation method.

## Input and output

`-i`, `--input`
:   The input raster. Any format that GDAL can read, in any spatial reference system, is accepted: GeoTIFF, ESRI
    ASCII grid, NetCDF, a VRT mosaic, and so on. The raster must be georeferenced. It is warped to the global geodetic
    (EPSG:4326) grid on the fly, so it does not need to be reprojected beforehand. The input can also be given as a
    positional argument.

`-o`, `--output-dir` (default: `terrain_tiles_qm`)
:   The output folder. It is created, with its parents, if it does not exist. `qm_tiler` writes the tiles there, in
    `<z>/<x>/<y>.terrain` files, and the `layer.json` tileset description.

## Zooms and tiles

The tiles follow the [TMS specification](https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification) on the global
geodetic grid. At zoom 0 the world is covered by two 180° × 180° tiles, and each zoom level splits every tile of the
previous one into four.

The pyramid is built **from the bottom up**: from the start zoom (deep, high resolution) to the end zoom (shallow).

`-s`, `--start-zoom` (default: `-1`)
:   The deepest zoom to build. It must be greater than or equal to the end zoom. If negative, `qm_tiler` uses the
    deepest zoom supported by the resolution of the raster.

`-e`, `--end-zoom` (default: `0`)
:   The shallowest zoom to build.

`--samples-per-tile` (default: `256`)
:   The number of samples per side taken from the raster for each tile. The raster is warped directly to this
    number of samples. TMS tiles normally have 256 × 256 samples. With a smaller value, the base mesh is coarser, and
    the TIN creation is much faster, at the cost of losing detail.

!!! tip "Building a pyramid in several runs"
    `qm_tiler` merges the zooms it creates into any `layer.json` already in the output folder, so you can split the
    work, for example to run each part on a different machine or to resume after an interruption:

    ```bash
    qm_tiler -i dem.tif -o tiles -s 12 -e 6
    qm_tiler -i dem.tif -o tiles -s 5 -e 0
    ```

    Use the same `--no-normals` setting in every run: the `layer.json` declares the normals extension according to
    the last run.

    Tiles at zoom \(z\) are built independently from those at other zooms, so the order of the runs does not
    matter.

## Interpreting the raster

These options change how the values of the raster are read, without modifying the source data.

`-b`, `--bathymetry`
:   A flag. When set, the raster is read as **depths** (positive downwards) instead of elevations, and the values are
    negated. In the EMODnet Bathymetry project, for which this tool was created, the DTMs contain depths.

    !!! warning "Changed in v0.3"
        `-b` is now a flag. Use `-b`, not `-b true`: `true` would be read as a second input file. In a configuration
        file, `bathymetry = true` still works.

`--clip-high`, `--clip-low` (default: `inf`, `-inf`)
:   Clip the values of the raster above / below these thresholds. The nodata cells are detected before clipping, so
    they are never turned into valid heights.

`--above-sea-level-scale-factor`, `--below-sea-level-scale-factor` (default: `-1`)
:   Exaggerate the heights above / below sea level (values greater / smaller than 0) by these factors. A negative
    value disables the scaling. Having two factors lets you, for example, exaggerate the seabed while keeping the land
    as is.

Each sample is processed in this order:

1. **Nodata** cells (and areas of the tile outside the raster) are set to height 0, i.e., sea level.
2. **Clipping** is applied to the raw value of the raster. With `-b`, this means that the thresholds are *depths*.
3. With `-b`, the value is **negated**.
4. The **scale factors** are applied, according to the sign of the resulting height.

## Normals

`--no-normals`
:   A flag. By default, the tiles include oct-encoded per-vertex normals (Cesium's `octvertexnormals` extension), and
    `layer.json` declares them. Cesium needs them to shade the terrain when lighting is enabled. The normals of the
    vertices on a shared border are the same in both tiles, so the shading shows no seams (except with the
    `delaunay` strategy, see [known issues](known-issues.md)). Use this flag to leave them out and get slightly
    smaller tiles.

See [Visualizing in Cesium](cesium.md) for the settings needed on the client side.

## TIN creation

`--tc-strategy` (default: `greedy`)
:   Selects the method used to create the mesh of each tile. All the options related to TIN creation start with
    `tc-`, followed by the identifier of the method. See [TIN creation](tin-creation/index.md).

## Processing

`--num-threads` (default: `0`) and `--scheduler` (default: `rowwise`) control how tiles are processed within a zoom,
and in parallel. See [Tile processing & threads](tile-processing.md).

## Debugging

`--debug-dir`
:   If set, the mesh of each tile is also written to this folder in [OFF](https://en.wikipedia.org/wiki/OFF_(file_format))
    format, in the same `<z>/<x>/<y>` layout. The OFF files can be opened in mesh viewers such as
    [MeshLab](https://www.meshlab.net/).

## Configuration file

`-c`, `--config`
:   Reads options from a file instead of the command line. Each line has the form `option = value`, using the long
    option names without the leading `--`. Options that take one value per zoom are repeated, one line per value.

```ini title="example.cfg"
start-zoom = -1
end-zoom = 0
bathymetry = true
samples-per-tile = 256
num-threads = 12
scheduler = 4connected
tc-strategy = greedy
tc-greedy-error-tol = 5120
```

You can combine a configuration file and command line options, for example to apply the same settings to several
inputs:

```bash
qm_tiler -c example.cfg -i dem_a.tif -o tiles_a
qm_tiler -c example.cfg -i dem_b.tif -o tiles_b
```

When an option appears in both places, **the command line takes precedence** over the configuration file.
