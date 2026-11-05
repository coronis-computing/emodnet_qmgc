# Greedy insertion

`--tc-strategy greedy` (the default) implements a slightly modified version of the algorithm described in:

> M. Garland and P. S. Heckbert. *Fast polygonal approximation of terrains and height fields.* Technical report
> CMU-CS-95-181, Carnegie Mellon University, 1995.

## How it works

1. Start from a very coarse triangulation of the tile in the XY plane. By default, this is the two triangles formed
   by the four corners of the tile (plus the border vertices fixed by the neighboring tiles already built).
2. For each remaining sample of the raster, compute the error between the current TIN and the sample.
3. Insert the sample with the largest error in the triangulation (Delaunay insertion), and update the errors of the
   samples covered by the new triangles.
4. Repeat until every error is below the tolerance.

The result is a mesh that is dense where the relief is rough and sparse where it is flat, with a **guaranteed
maximum error**.

## Parameters

`--tc-greedy-error-tol` (\*) (default: `150000`)
:   The maximum error allowed, in meters, between the TIN and the raster samples. This is the main parameter of the
    method. With a single value, it is the tolerance at zoom 0, and it is halved at each zoom (see
    [parameters per zoom](../parameters-per-zoom.md)). The default gives about 146 m at zoom 10.

    To choose it, decide the error you accept at your deepest zoom \(z\), and multiply it by \(2^z\). For
    instance, 5 m at zoom 10 is `--tc-greedy-error-tol 5120`.

`--tc-greedy-init-grid-size` (\*) (default: `64 32 16 0`)
:   The number of cells per side of a regular grid used as the initial mesh, instead of just the four corners. A
    value `<= 0` uses only the four corners.

    The grid sets a **minimum density** for the mesh. This matters at the shallowest zooms. There, a tile covers a
    large part of the globe. The error is only measured in height, so a flat area could be covered with a few huge
    triangles that cut straight through the curvature of the Earth. The default is a 2.8° grid at zooms 0, 1 and 2
    (64, 32 and 16 cells), and no grid for deeper zooms. Set it to `0` to disable it.

    !!! warning
        A single value is halved at each zoom: `--tc-greedy-init-grid-size 8` means 8 cells at zoom 0, 4 at zoom 1,
        and so on. To use the same grid at every zoom, give the value twice: `--tc-greedy-init-grid-size 8 8`.

`--tc-greedy-error-type` (default: `height`)
:   How the error is measured: `height` (the vertical distance between the sample and the TIN), or `3d` (the
    3D distance). The `3d` option is experimental, and we do not recommend changing the default.

## Examples

Error of 5 m at zoom 10 (and 10 m at zoom 9, 20 m at zoom 8...):

```bash
qm_tiler -i dem.tif -o tiles --tc-greedy-error-tol 5120
```

Explicit tolerances for the first zooms, and 2 m for zoom 4 and deeper:

```bash
qm_tiler -i dem.tif -o tiles --tc-greedy-error-tol 20000 5000 500 50 2
```
