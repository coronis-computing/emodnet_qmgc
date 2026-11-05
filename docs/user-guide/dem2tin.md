# dem2tin

`dem2tin` runs only the **TIN creation** step, on a whole input, without building a pyramid of tiles. It writes the
resulting triangle mesh to an [OFF](https://en.wikipedia.org/wiki/OFF_(file_format)) file. It is useful to try the
[TIN creation methods](tin-creation/index.md) and their parameters, and to look at the results in a mesh viewer such
as [MeshLab](https://www.meshlab.net/).

```bash
dem2tin -i <input> -o <output.off> [options]
```

Most options are the same as in `qm_tiler`, with two differences:

- There are no zoom levels, so every parameter takes a **single value**, with no per-zoom scaling. The defaults are
  therefore much smaller than in `qm_tiler`.
- The input can also be a point set or a mesh, besides a GDAL raster.

## Inputs

`--input-type` selects the type of the input (`-i`):

| `--input-type` | Input |
|----------------|-------|
| `gdal` (default) | A GDAL raster. |
| `point-set` | A text file with the three coordinates of a point on each line. |
| `off` | A TIN in an OFF file. |

The point sets and TINs do not need to be regular grids. However, the points must be projectable to the XY plane
(i.e., a height field), and in metric coordinates.

The `ps-*` methods work in ECEF coordinates. For GDAL inputs, `dem2tin` has no direct way of converting the raster's
coordinates to ECEF. Use the point set methods with `point-set` or `off` inputs.

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `-i`, `--input` | | The input file (also accepted as a positional argument). |
| `--input-type` | `gdal` | `gdal`, `point-set` or `off`. |
| `-o`, `--output` | `./out.off` | The output OFF file. |
| `-b`, `--bathymetry` | off | Flag: the raster contains depths (GDAL inputs only). |
| `--clip-high`, `--clip-low` | `inf`, `-inf` | Clip the raster values (GDAL inputs only). |
| `--reset-origin` | `true` | Moves the origin of the data to the minimum corner of its bounding box (non-GDAL inputs only). |
| `--export-new-origin` | | File where the new origin (minimum X, Y, Z) is written, when `--reset-origin` is set. Adding it to the output vertices gives the original coordinates. |
| `--tc-strategy` | `greedy` | `greedy`, `lt`, `delaunay`, `ps-hierarchy`, `ps-wlop`, `ps-grid`, `ps-random`. |
| `--tc-greedy-error-tol` | `0.1` | See [greedy insertion](tin-creation/greedy.md). |
| `--tc-greedy-init-grid-size` | `-1` | |
| `--tc-greedy-error-type` | `height` | |
| `--tc-lt-stop-edges-count` | `500` | See [edge collapse](tin-creation/edge-collapse.md). |
| `--tc-lt-weight-volume` / `-boundary` / `-shape` | `0.5` / `0.5` / `1e-10` | |
| `--tc-ps-border-max-error` | `0.01` | See [point set simplification](tin-creation/point-set.md). |
| `--tc-ps-border-max-length-xy-percent` | `10` | |
| `--tc-ps-features-min-size` | `5` | |
| `--tc-ps-preserve-sharp-edges` | `true` | |
| `--tc-ps-hierarchy-cluster-size` | `100` | |
| `--tc-ps-hierarchy-max-surface-variance` | `0.01` | |
| `--tc-ps-wlop-retain-percent` | `5` | |
| `--tc-ps-wlop-radius` | `0.2` | |
| `--tc-ps-wlop-iter-number` | `35` | |
| `--tc-ps-grid-cell-size` | `0.1` | |
| `--tc-ps-random-percent` | `80` | |
| `--verbose` | `true` | Prints progress on the screen. |
| `-c`, `--config` | | Configuration file with `option = value` lines. |

## Why reset the origin?

Coordinates such as UTM eastings or ECEF positions are large numbers. Many 3D viewers store vertices in single
precision, so a mesh in its original coordinates may show precision artifacts, or be hard to navigate. With
`--reset-origin` (the default), the mesh is moved so that the minimum corner of its bounding box is at the origin. Use
`--export-new-origin` to keep the offset and move the mesh back later.

## Example

```bash
# Greedy insertion on a point cloud, with a 0.5 m tolerance
dem2tin -i survey.xyz --input-type point-set -o survey_tin.off \
    --tc-greedy-error-tol 0.5 --export-new-origin survey_origin.txt
```
