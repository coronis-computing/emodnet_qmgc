# Tutorial

This tutorial goes through the whole process: preparing a digital terrain model (DTM), creating its quantized-mesh
tiles, and viewing them in Cesium.

We assume that you have [installed](installation.md) the tools, and that `qm_tiler` and the scripts in the `scripts`
folder are in your `PATH` (otherwise, use their full path in the commands below). If you use Docker, prefix the
commands with `docker run --rm -v "$PWD":/data emodnet_qmgc`.

## Input data

The input to `qm_tiler` can be any properly georeferenced raster that GDAL can read.

In this tutorial we use a part of the DTM built in the EMODnet Bathymetry project, which covers the seabed of the
European seas. You can download it from the [EMODnet Bathymetry](https://emodnet.ec.europa.eu/en/bathymetry) portal.
The DTM is split into tiles, available in several formats.

In this example we downloaded tile *F3* in ESRI ASCII format. `gdalinfo` shows that this format does not set the
spatial reference system (the `Coordinate System is` line is empty):

```console
$ gdalinfo F3_2018.asc
Driver: AAIGrid/Arc/Info ASCII Grid
Files: F3_2018.asc
       F3_2018.asc.aux.xml
Size is 9504, 9024
Coordinate System is `'
Origin = (-16.262500050000000,43.137500140080000)
Pixel Size = (0.001041666670000,-0.001041666670000)
Corner Coordinates:
Upper Left  ( -16.2625000,  43.1375001)
Lower Left  ( -16.2625000,  33.7375001)
Upper Right (  -6.3625000,  43.1375001)
Lower Right (  -6.3625000,  33.7375001)
Center      ( -11.3125000,  38.4375001)
Band 1 Block=9504x1 Type=Float32, ColorInterp=Undefined
  NoData Value=99
```

We know that the grid uses WGS84 geographic coordinates, so we assign [EPSG:4326](https://epsg.io/4326) to it, and
convert it to a GeoTIFF for convenience:

```bash
gdal_translate -a_srs EPSG:4326 F3_2018.asc F3_2018.tif
```

Optionally, build overviews. They speed up the low zoom levels a lot on large rasters:

```bash
gdaladdo -r average F3_2018.tif
```

!!! tip "Mosaics of several files"
    You can merge several DTM tiles into a single input with a [VRT](https://gdal.org/drivers/raster/vrt.html), without
    copying the data:

    ```bash
    gdalbuildvrt merged.vrt *.tif
    ```

    If you georeference all the tiles in the portal and follow this tutorial with the resulting VRT, you get results
    similar to those in the EMODnet Bathymetry 3D viewer.

## Create the quantized-mesh tiles

The first decision is the **TIN creation method**. The methods available are described in
[TIN creation](../user-guide/tin-creation/index.md):

- [greedy insertion](../user-guide/tin-creation/greedy.md) (`greedy`, the default);
- [edge collapse](../user-guide/tin-creation/edge-collapse.md) (`lt`);
- four [point set simplification](../user-guide/tin-creation/point-set.md) methods (`ps-*`);
- a plain Delaunay triangulation of all the samples (`delaunay`).

During the EMODnet Bathymetry project we tested all of them on our data. **Greedy insertion** gave the most intuitive
way of tuning the quality of the approximation, and is reasonably fast, so we use it here.

Its key parameter is `--tc-greedy-error-tol`: the maximum difference in height allowed between the TIN and the
raster. This parameter is [set per zoom](../user-guide/parameters-per-zoom.md). If you give a single value \(p_0\),
it is the tolerance at zoom 0, and the tolerance at zoom \(z\) is

\[
p_z = \frac{p_0}{2^z}
\]

The resolution of our data supports zooms up to about 10. Say we want an error of at most 5 meters at zoom 10. The
value for zoom 0 is then:

\[
p_0 = 5 \cdot 2^{10} = 5120
\]

Now run `qm_tiler`:

```bash
qm_tiler -i F3_2018.tif -o qm_tiles --tc-greedy-error-tol 5120 -b
```

A few notes about this command:

- We left out `--tc-strategy`, because `greedy` is the default.
- `-b` (`--bathymetry`) tells `qm_tiler` that the raster contains **depths** instead of elevations.
- The default `--tc-greedy-init-grid-size` adds a base grid at zooms 0 to 2, which preserves the curvature of the
  globe on these large tiles. Keep the default unless you have a reason to change it (see
  [greedy insertion](../user-guide/tin-creation/greedy.md)).
- All the CPU cores are used by default (`--num-threads 0`).

When it finishes, `qm_tiler` writes the `layer.json` file that Cesium needs into `qm_tiles`. The output looks like
this:

```text
qm_tiles/
├── layer.json
├── 0/
│   └── 0/
│       └── 0.terrain
├── 1/
│   └── ...
└── 10/
    └── <x>/
        └── <y>.terrain
```

The tiles follow the [TMS](https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification) layout (`z/x/y.terrain`) on
the global geodetic grid, and are stored gzip-compressed.

### Other useful variants

Build only part of the pyramid, here zooms 5 to 1:

```bash
qm_tiler -i F3_2018.tif -o qm_tiles -s 5 -e 1 --tc-greedy-error-tol 5120 -b
```

You can build a large pyramid in several runs on the same output folder. `qm_tiler` merges the zooms it creates with
those already listed in `layer.json`:

```bash
qm_tiler -i F3_2018.tif -o qm_tiles -s 10 -e 6 --tc-greedy-error-tol 5120 -b
qm_tiler -i F3_2018.tif -o qm_tiles -s 5  -e 0 --tc-greedy-error-tol 5120 -b
```

Take fewer samples per tile to speed up the TIN creation, at the cost of a coarser base mesh:

```bash
qm_tiler -i F3_2018.tif -o qm_tiles --samples-per-tile 64 --tc-greedy-error-tol 5120 -b
```

Make the results reproducible by using a single thread. When several threads are used, the order in which the tiles are
built changes the shape of their borders (see [tile processing](../user-guide/tile-processing.md)):

```bash
qm_tiler -i F3_2018.tif -o qm_tiles --tc-greedy-error-tol 5120 -b --num-threads 1
```

Keep all the options in a [configuration file](../user-guide/general-parameters.md#configuration-file):

```ini title="emodnet.cfg"
bathymetry = true
tc-strategy = greedy
tc-greedy-error-tol = 5120
num-threads = 0
scheduler = 4connected
```

```bash
qm_tiler -c emodnet.cfg -i F3_2018.tif -o qm_tiles
```

### Reproducing the heightmap-1.0 format

Before quantized meshes, Cesium represented terrain as regular grids, in the
[heightmap-1.0](https://github.com/CesiumGS/cesium/wiki/heightmap-1.0-terrain-format) format. You can produce the
same regular triangulation, encoded as quantized-mesh, with:

```bash
qm_tiler -i F3_2018.tif -o qm_tiles --samples-per-tile 64 --tc-strategy delaunay -b
```

The `delaunay` strategy triangulates the samples in the XY plane, like Cesium does on the client side with heightmap
tiles. The heightmap format limits the depths it can represent, so it cannot store our bathymetry. The quantized-mesh
format has no such limit.

## Serve the tiles

Cesium fetches the tiles over HTTP. They must be served with CORS headers and with `Content-Encoding: gzip` (the
`.terrain` files are stored gzip-compressed). For testing, use the bundled server:

```bash
simple_tiles_server.py -p 8000 qm_tiles
```

For production, use a regular web server configured in the same way (see
[Visualizing in Cesium](../user-guide/cesium.md#serving-the-tiles-in-production)).

## View the tiles in Cesium

Save this page as `viewer.html` and open it in a browser. It reads the tiles from the local server, and needs no
Cesium ion account:

```html title="viewer.html"
--8<-- "docs/assets/viewer.html"
```

Our terrain provider points to the local server. It requests the vertex normals, which Cesium uses to shade the
terrain, and exaggerates the heights by 5 to enhance the relief.

You now have your own bathymetry in a virtual globe. See the [user guide](../user-guide/qm_tiler.md) for every option
of `qm_tiler`.
