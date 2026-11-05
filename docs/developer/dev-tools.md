# Developer tools

The `src/dev-tests` folder contains small tools written during development, to test parts of the main apps. Some of
them may also be useful on their own, for example to inspect or validate a tileset.

They are built by default. Disable them with `cmake .. -DQMGC_BUILD_DEV_TESTS=OFF`. They are not installed by
`make install`, so run them from the `build` folder.

## Inspecting and validating tiles

| Tool | Description |
|------|-------------|
| `qm_parser` | Parses a `.terrain` file and prints its contents (header, vertices, indices, edges, extensions) as plain text. |
| `test_read_write` | Reads a `.terrain` file, writes it back, and reads it again, to check that the I/O round-trips. |
| `test_check_borders` | Checks that the vertices on the borders shared by neighboring tiles match, across a whole tileset. |
| `compute_statistics` | Compares the tiles of one zoom (or a single tile) with the source raster, and writes per-tile and global statistics (e.g., number of vertices, approximation error) to a MATLAB `.m` file. `matlab/loadStats.m` loads these files for several methods and zooms, to compare them. |
| `get_tile_bounds` | Prints the geographic bounds of a tile, given its zoom and coordinates. |

Run any of them with `--help` for its options.

## Experiments with CGAL

| Tool | Description |
|------|-------------|
| `simplify_preserving_border_edges` | Simplifies a mesh with CGAL's edge collapse, keeping the edges on the border. |
| `hierarchy_simplification` | Simplifies a mesh with CGAL's hierarchy point set simplification, keeping the edges on the border. |
| `wlop_simplification` | Simplifies a point set with WLOP, and triangulates it in the 2D plane. |
| `remesh_preserving_borders` | Remeshes a polyhedral domain, keeping its borders. |
| `detect_features_without_borders` | Detects the sharp edges of a mesh, excluding the border edges. |
