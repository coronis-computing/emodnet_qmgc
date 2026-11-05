# Known issues & troubleshooting

## Known issues

**Faint lighting seams with the `delaunay` strategy.**
:   With `delaunay`, neighboring tiles do not share their borders, so each tile computes the normals of its border
    vertices on its own. This may show as faint lighting seams along the tile borders. The other strategies reuse the
    normals of the neighboring tiles, so their shading is continuous.

**Slow low zoom levels on very large DEMs.**
:   A tile at a low zoom level covers a large part of the raster, which has to be resampled. GDAL uses the overviews
    of the dataset automatically when available, so build them beforehand: `gdaladdo -r average <input_raster>`.

**Results are not deterministic with several threads.**
:   This is expected. See [Tile processing](tile-processing.md#how-the-threads-are-scheduled). Use
    `--num-threads 1` for reproducible results.

**Tiles differ from those created with versions before v0.3.**
:   This is expected. The raster is now sampled at the tile's nodes, nodata is no longer averaged into valid samples,
    and `--samples-per-tile` warps the raster directly. See the [changelog](../changelog.md).

## Troubleshooting

**"too many positional options" error when using `-b true`.**
:   `-b` is a flag since v0.3. Use `-b` alone.

**Cesium shows nothing, or a flat globe.**
:   - Check the browser's developer console for CORS errors, and use a server that sets the CORS headers (see
      [Serving the tiles](cesium.md#serving-the-tiles-for-testing)).
    - Check that the `.terrain` files are served with `Content-Encoding: gzip`. Without it, Cesium receives
      compressed data and fails to decode the tiles.
    - Check that `layer.json` is in the root of the URL given to `CesiumTerrainProvider`.

**The terrain is not shaded.**
:   Set `requestVertexNormals: true` in the terrain provider and `viewer.scene.globe.enableLighting = true`. Check that
    the tiles were not created with `--no-normals`, and that `layer.json` lists `"octvertexnormals"`.

**Land is shown below sea level and the sea above it.**
:   The raster stores depths, not elevations. Add `-b`. Conversely, remove `-b` if the raster stores elevations.

**Large flat areas at sea level.**
:   Nodata cells, and areas of the tiles outside the raster, are set to height 0. Make sure that the raster declares
    its nodata value (check with `gdalinfo`), and use `--clip-low` / `--clip-high` to remove spurious values.

**The tiles at zooms 0-2 cut through the globe.**
:   The tiles at these zooms are huge, and need a minimum density to follow the curvature of the Earth. With the
    greedy strategy, keep the default `--tc-greedy-init-grid-size` (`64 32 16 0`), or increase it.

**"Could not parse the existing layer.json".**
:   The output folder contains an invalid `layer.json` (e.g., from an interrupted copy). Delete it and run
    `qm_tiler` again, or recreate it with [`create_layer_file.py`](scripts.md#create_layer_filepy).

**Swapped coordinates, or the data appears in the wrong place.**
:   Make sure that the input's spatial reference system is set correctly (`gdalinfo`). If it is missing, assign it
    with `gdal_translate -a_srs EPSG:xxxx` or describe it in a VRT.
