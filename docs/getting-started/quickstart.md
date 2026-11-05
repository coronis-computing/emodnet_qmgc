# Quick start

This page gets you from a raster to terrain in Cesium as fast as possible. The [tutorial](tutorial.md) goes through the
same steps in more detail.

## 1. Check your input

`qm_tiler` accepts any **georeferenced** raster that GDAL can read. Make sure that GDAL knows its spatial reference
system:

```bash
gdalinfo my_dem.tif
```

If the `Coordinate System is` line is empty, set the SRS first (see
[Input data](tutorial.md#input-data) in the tutorial).

!!! tip "Large rasters"
    For large DEMs, build overviews beforehand. GDAL uses them automatically when sampling the low zoom levels, which
    is much faster:

    ```bash
    gdaladdo -r average my_dem.tif
    ```

## 2. Create the tiles

```bash
qm_tiler -i my_dem.tif -o tiles
```

If the raster stores **depths** (positive downwards) instead of elevations, add `-b`:

```bash
qm_tiler -i my_dem.tif -o tiles -b
```

By default, `qm_tiler`:

- builds every zoom level, from the deepest one supported by the raster's resolution up to zoom 0;
- uses the [greedy insertion](../user-guide/tin-creation/greedy.md) TIN creation method;
- uses all the CPU cores;
- writes `tiles/layer.json`, the tileset description that Cesium needs.

## 3. Serve the tiles

Cesium needs the tiles to be served over HTTP with CORS and gzip headers. The bundled test server sets them:

```bash
python3 scripts/simple_tiles_server.py -p 8000 tiles
```

## 4. View them in Cesium

Create a CesiumJS viewer with a terrain provider that points to the server:

```js
const viewer = new Cesium.Viewer("cesiumContainer", {
  terrainProvider: await Cesium.CesiumTerrainProvider.fromUrl("http://localhost:8000/", {
    requestVertexNormals: true,
  }),
});
viewer.scene.globe.enableLighting = true;
```

A complete HTML page is on the [Visualizing in Cesium](../user-guide/cesium.md) page.

## Next steps

- Tune how much the meshes are simplified: [greedy insertion parameters](../user-guide/tin-creation/greedy.md).
- Learn how to [set parameters per zoom level](../user-guide/parameters-per-zoom.md).
- Build a large pyramid [in several runs](../user-guide/general-parameters.md#zooms-and-tiles).
