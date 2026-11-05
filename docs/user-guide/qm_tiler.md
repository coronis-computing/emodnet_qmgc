# qm_tiler

`qm_tiler` is the main tool of the project. It creates the multiresolution pyramid of terrain tiles from a GDAL
raster. Each tile is a Triangulated Irregular Network (TIN) in Cesium's
[quantized-mesh-1.0](https://github.com/CesiumGS/quantized-mesh) format.

```bash
qm_tiler -i <input_raster> -o <output_tiles_dir> [options]
```

The input can also be given as a positional argument: `qm_tiler <input_raster> -o <output_tiles_dir>`.

## What it produces

- One `.terrain` file per tile, in a [TMS](https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification) folder
  layout (`<z>/<x>/<y>.terrain`) on the global geodetic (EPSG:4326) grid. Zoom 0 has two tiles, one per hemisphere
  (west and east of the 0° meridian). The files are gzip-compressed, and include the oct-encoded per-vertex normals
  extension unless `--no-normals` is set.
- A `layer.json` file in the output folder, describing the tiles available at each zoom. If the folder already has a
  `layer.json`, the zooms it lists are kept, so the pyramid can be built in several runs.

## Options at a glance

The options are explained in detail in the pages that follow. Options marked with (\*) accept
[one value per zoom level](parameters-per-zoom.md).

### General

| Option | Default | Description |
|--------|---------|-------------|
| `-h`, `--help` | | Prints the help message. |
| `-i`, `--input` | | Input raster, in any format supported by GDAL. |
| `-o`, `--output-dir` | `terrain_tiles_qm` | Output folder for the tiles and `layer.json`. |
| `-s`, `--start-zoom` | `-1` | Deepest zoom level to build (the pyramid is built from the bottom up). `< 0` means the deepest zoom supported by the raster's resolution. |
| `-e`, `--end-zoom` | `0` | Shallowest zoom level to build. |
| `-b`, `--bathymetry` | off | Flag: the raster contains depths instead of elevations. |
| `--no-normals` | off | Flag: do not add the per-vertex normals extension. |
| `--samples-per-tile` | `256` | Samples per side taken from the raster for each tile. |
| `--clip-high` | `inf` | Clips the heights above this value. |
| `--clip-low` | `-inf` | Clips the heights below this value. |
| `--above-sea-level-scale-factor` | `-1` | Scale factor for heights above sea level (ignored if `< 0`). |
| `--below-sea-level-scale-factor` | `-1` | Scale factor for heights below sea level (ignored if `< 0`). |
| `--num-threads` | `0` | Number of tiles built in parallel (`0` = all the cores). |
| `--scheduler` | `rowwise` | Preferred order of the tiles within a zoom: `rowwise`, `columnwise`, `chessboard`, `4connected`. |
| `--debug-dir` | | Also writes each tile's mesh to this folder, in OFF format. |
| `-c`, `--config` | | Configuration file with `option = value` lines. |

See [General parameters](general-parameters.md) and [Tile processing](tile-processing.md).

### TIN creation

| Option | Default | Description |
|--------|---------|-------------|
| `--tc-strategy` | `greedy` | TIN creation method: `greedy`, `lt`, `delaunay`, `ps-hierarchy`, `ps-wlop`, `ps-grid`, `ps-random`. |

**Greedy insertion** (`greedy`), see [details](tin-creation/greedy.md):

| Option | Default | Description |
|--------|---------|-------------|
| `--tc-greedy-error-tol` (\*) | `150000` | Maximum height error allowed at zoom 0, in meters. |
| `--tc-greedy-init-grid-size` (\*) | `64 32 16 0` | Cells per side of the initial grid, per zoom. `<= 0` uses only the 4 corners of the tile. |
| `--tc-greedy-error-type` | `height` | Error measure: `height` or `3d` (experimental). |

**Edge collapse** (`lt`), see [details](tin-creation/edge-collapse.md):

| Option | Default | Description |
|--------|---------|-------------|
| `--tc-lt-stop-edges-count` (\*) | `500` | Simplification stops when the mesh has fewer edges than this. |
| `--tc-lt-weight-volume` | `0.5` | Volume weight of the Lindstrom-Turk cost. |
| `--tc-lt-weight-boundary` | `0.5` | Boundary weight of the Lindstrom-Turk cost. |
| `--tc-lt-weight-shape` | `1e-10` | Shape weight of the Lindstrom-Turk cost. |

**Point set simplification** (`ps-*`), see [details](tin-creation/point-set.md):

| Option | Default | Description |
|--------|---------|-------------|
| `--tc-ps-border-max-error` (\*) | `10000` | Maximum height error when simplifying the tile borders and the sharp edges. |
| `--tc-ps-border-max-length-xy-percent` (\*) | `20` | Maximum length of a border edge in the XY plane, as a percentage of the tile size. |
| `--tc-ps-preserve-sharp-edges` | `true` | Preserves the sharp edges of the terrain. |
| `--tc-ps-features-min-size` | `5` | Minimum number of points in a sharp-edge polyline to keep it. |
| `--tc-ps-hierarchy-cluster-size` (\*) | `1000` | `ps-hierarchy`: maximum cluster size. |
| `--tc-ps-hierarchy-max-surface-variance` (\*) | `1000` | `ps-hierarchy`: maximum surface variation. |
| `--tc-ps-wlop-retain-percent` (\*) | `1` | `ps-wlop`: percentage of points to keep. |
| `--tc-ps-wlop-radius` (\*) | `10000` | `ps-wlop`: neighborhood radius. |
| `--tc-ps-wlop-iter-number` | `35` | `ps-wlop`: number of iterations. |
| `--tc-ps-grid-cell-size` (\*) | `10000` | `ps-grid`: cell size. |
| `--tc-ps-random-percent` (\*) | `90` | `ps-random`: percentage of points to remove. |

!!! note "The built-in help"
    `qm_tiler --help` prints the same list. In the help text, (\*) does not appear next to `--tc-ps-wlop-radius` and
    `--tc-ps-random-percent`, but both options also accept one value per zoom.

## Exit status and messages

`qm_tiler` prints the zoom and tile being processed. It ends with the time taken and the path of the `layer.json`
it wrote. On any error (invalid option, unreadable raster, failed write, and so on), it prints a message starting
with `[ERROR]` and exits with status 1.
