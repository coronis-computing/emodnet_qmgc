# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Removed

- **Dependency on Cesium Terrain Builder (libctb).** The project no longer needs libctb to build or run. The parts of it that were used now live in this repository, under `src/base/tiling/` (namespace `qmgc`).
- `cmake/modules/FindCTB.cmake`.
- `docker/cesium-terrain-builder/patches/GDALTiler.cpp.patch`, plus the steps in the Dockerfile that built GDAL 2.1 and libctb from source.

### Added

- `src/base/tiling/geo_types.h`: `TilePoint`, `TileCoordinate`, `TileBounds`, `CRSPoint` and `CRSBounds`. These are derived from libctb's types and replace them.
- `src/base/tiling/global_geodetic_grid.h`: `GlobalGeodeticGrid`, the TMS global-geodetic (EPSG:4326) tiling grid. It replaces `ctb::Grid` / `ctb::GlobalGeodetic`.
- `src/base/tiling/raster_tiler.h/.cpp`: `RasterTiler`, which extracts the piece of the source raster covered by one tile. It replaces `ctb::GDALTiler` / `ctb::TerrainTiler`. The raster is warped with GDAL's own `GDALWarp()`, so GDAL now takes care of choosing overviews, handling nodata and reprojecting.
- `src/base/tiling/qmgc_exception.h`: `QmgcException`, which replaces `ctb::CTBException`.
- New raster sampling options in `qmgc::TilerOptions`:
  - `samplingMode`:
    - `NodeRegistered` (the default) places samples exactly on the tile's node grid.
    - `HeightmapBorder` reproduces libctb's sampling window.
  - `excludeNoDataFromResampling` declares the band's nodata value to the warper, so nodata is not averaged into valid samples.
  - `resampleAlg`, `useOverviews`, `errorThreshold` and `warpThreads` expose the matching `gdalwarp` settings.

### Changed

- **Generated tiles differ from earlier versions.** Expect differences when comparing against previously generated tile sets. Together with the fixes listed under *Fixed*, three changes cause this:
  - `qm_tiler` and `compute_statistics` now sample the raster at the tile's nodes (`SamplingMode::NodeRegistered`). libctb's sampling window left the terrain displaced by half a sample towards the south-east.
  - The nodata value is now declared to the warper, so nodata is no longer averaged into valid samples near the edges of the data.
  - The `--samples-per-tile` option now warps the raster directly to the requested number of samples. Previously, a 256×256 tile was warped and then decimated by GDAL's `RasterIO`, which did not keep the first and last samples on the tile's edges.
- `QuantizedMeshTiler` now *contains* a `qmgc::RasterTiler` instead of inheriting from `ctb::TerrainTiler`. As a result:
  - The internal mutex and the hand-written copy constructor are gone. Each worker thread already had its own tiler, so the mutex protected nothing.
  - Warped raster tiles are released automatically, through a `std::unique_ptr` with a GDAL deleter, even when an exception is thrown. Previously, an exception leaked the tile and left the mutex locked.
  - The tile height buffer is a `std::vector` instead of a variable-length array. Variable-length arrays are not standard C++, and this one put about 256 KB on the stack.
  - The error raised when the raster cannot be read now names the tile (`z/x/y`).
- `QuantizedMeshTile` now derives from `qmgc::TileCoordinate` instead of `ctb::Tile`.
- The pyramid builder, tile schedulers and border-vertices cache now use the `qmgc` tile types.
- The dev-tests (`qm_parser`, `test_read_write`, `test_check_borders`, `compute_statistics`) now use the `qmgc` types.
- **Ported to CGAL 5.x** (tested with 5.6). This drops support for CGAL 4.x:
  - `Projection_traits_3.h` is now included from `CGAL/Kernel_23/internal/`.
  - `further_constrained_placement.h` no longer uses the removed `Detail/Common.h`. It also uses `boost::optional` and `Edge_profile::Triangle_mesh`.
  - `Count_stop_predicate` is now `Edge_count_stop_predicate`, and `LindstromTurk_params` is now `SMS::internal::LindstromTurk_params`.
  - The grid, hierarchy, random and WLOP point-set simplifiers now pass a point range and named parameters, as CGAL 5 requires. This includes the `hierarchy_simplification` and `wlop_simplification` dev-tests.
- `CMAKE_BUILD_TYPE` now defaults to `Release`. Previously, a plain `cmake ..` (including the one in the Dockerfile) built without optimization, which makes the CGAL code run many times slower.
- `ZoomTilesBorderVerticesCache::getConstrainedBorderVerticesForTile` and `setConstrainedBorderVerticesForTile` now return `void`. Both were declared `bool` but returned nothing, which is undefined behaviour, and no caller used the result.
- **Build requirements:**
  - C++14 is required (CGAL 5 needs it) instead of C++11. It is set through `CMAKE_CXX_STANDARD`.
  - CMake 3.10 or later is required.
  - GDAL 3.0 or later is required.
- **Docker image:** now based on Ubuntu 24.04 and uses the distribution's `libgdal-dev` / `libcgal-dev` / `libgeographiclib-dev` packages.
- **GeographicLib is required again**, for the ECEF conversions. `FindGeographicLib.cmake` now also finds GeographicLib 2.x, whose library was renamed from `Geographic` to `GeographicLib`.
- `Readme.md` and `DeveloperNotes.md` are updated to match.
- The `test_check_borders` and `compute_statistics` dev-tests now handle zoom levels that have no tiles in `layer.json`.
- `compute_statistics` shares a single tiler (and GDAL dataset) across its OpenMP threads. libctb's tiler serialized raster reads with an internal mutex, which is gone, so the read is now serialized with an OpenMP critical section instead.
- The new `get_tile_bounds` dev-tool uses the `qmgc` grid instead of libctb.

### Fixed

- **Tile headers ignored the terrain's height.** Cesium uses the header's bounding sphere, center and horizon occlusion point to cull tiles and choose the level of detail. These were computed using the normalized height of each vertex (between 0 and 1) as if it were meters, which described a flat surface lying on the ellipsoid regardless of the actual relief. With significant relief, tiles could be culled while still visible, or refined at the wrong distance. Heights are now converted back to meters first, and the latitude/longitude of each vertex is kept in double precision (as floats, it lost about a meter).
- **Nodata cells could become valid heights.** Two problems:
  - Heights were clipped (`--clip-low` / `--clip-high`) before being compared with the nodata value, so a nodata value outside the clipping range became a valid-looking height at the clipping limit.
  - Heights read as `float` were compared with the `double` nodata value, which never matches when that value is not exactly representable as a float (e.g., `-99999.9`).

  Nodata is now detected before clipping, compared in float precision, and only when the band actually declares a nodata value (including NaN).
- **Two `Projection_traits_3_extended` functions recursed forever.** The `(Point_3, Point_3)` and `(Line_3, Point_3)` overloads of the squared distance functor called themselves instead of the base class. The base overloads are now brought into scope with a `using` declaration.
- **`create_layer_file.py` failed under Python 3**, which is the only Python in the Docker image. It read the result of `map()` twice, so the second read was empty and `max()` raised an error. It also:
  - wrote an `available` list that did not start at zoom 0 when the pyramid's minimum zoom was above 0. Cesium indexes that list by zoom level, so it now starts at zoom 0, with empty entries for levels without tiles.
  - took each zoom level's Y range from its first X folder only. It is now gathered over all the X folders.

- **Swapped axes with GDAL 3.** The target EPSG:4326 spatial reference now forces the traditional longitude/latitude axis order (`OAMS_TRADITIONAL_GIS_ORDER`). Without it, GDAL 3 returns coordinates with latitude and longitude swapped, which libctb does not handle.
- **Dataset extent with curved projections.** The source extent is now computed with `GDALSuggestedWarpOutput2`, which samples along the dataset's edges as well as its corners. Previously, only the four corners were transformed, which could leave parts of the dataset outside the tiled area for projections such as polar stereographic.
- **Out-of-bounds neighbouring tiles were accepted.** Tile indices are now signed. With unsigned indices, a neighbour at `x - 1 = -1` wrapped around to a huge positive value and passed the bounds checks in the schedulers and the border-vertices cache.
- **Pixel overflow at deep zoom levels.** Pixel indices in the grid are now 64-bit. libctb's 32-bit indices overflow beyond zoom level ~22.
- **`--tc-ps-wlop-iter-number` had no effect.** The option was read but never passed to CGAL. Default runs are unaffected, because the option's default and CGAL's default are both 35.

### Known issues

- `scripts/qm_tiler_large_files.py` is Python 2 only, so it does not run in the Docker image. It also still calls the `ctb-tile` command-line tool to produce intermediate GeoTIFF tiles. That tool is no longer installed in the Docker image, so the script only works where Cesium Terrain Builder is installed separately.
