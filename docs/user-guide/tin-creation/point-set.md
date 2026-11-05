# Point set simplification

The `ps-*` strategies use point set simplification algorithms from
[CGAL](https://doc.cgal.org/latest/Point_set_processing_3/index.html#title19), adapted to work at tile level.

Point set methods ignore the connectivity between the samples, which makes it hard to keep the square shape of the
tile in the XY plane, and its borders aligned with the neighboring tiles. For this reason, the simplification is split
into two steps:

1. **Borders and sharp edges.** The borders of the tile, and optionally the sharp edges of the terrain, are extracted
   as polylines and simplified (see [below](#step-1-borders-and-sharp-edges)).
2. **Interior points.** The rest of the samples are simplified with the selected point set algorithm. The result is
   then triangulated, together with the polylines from step 1.

Since both steps are independent, any point set simplification algorithm can be used in the second step.

## Step 1: borders and sharp edges

The polylines are simplified coarse to fine: starting from the original polylines, points are removed one at a time.
The algorithm was designed for 2D curves, but our polylines are in 3D, so we use a hybrid 2D-3D approach. The
simplification, and the preservation of the topology, work in the XY plane, as in the original method. The error of
removing a point is measured in Z, i.e., as an error in height.

These options apply to all the `ps-*` strategies:

`--tc-ps-border-max-error` (\*) (default: `10000`)
:   The maximum height error allowed when simplifying the polylines.

`--tc-ps-border-max-length-xy-percent` (\*) (default: `20`)
:   The maximum length of an edge of the polylines, projected to the XY plane, as a percentage `[0..100]` of the tile
    size.

`--tc-ps-preserve-sharp-edges` (default: `true`)
:   Detects, preserves and simplifies the sharp edges of the terrain: the edges where the dihedral angle between the
    incident faces is larger than 60°.

`--tc-ps-features-min-size` (default: `5`)
:   The minimum number of points in a sharp-edge polyline for it to be kept.

## Step 2: interior points

### Hierarchy (`ps-hierarchy`)

Splits the points recursively into clusters, until each cluster is small enough and flat enough. Each cluster is then
replaced by a single point.

`--tc-ps-hierarchy-cluster-size` (\*) (default: `1000`)
:   The maximum number of points in a cluster.

`--tc-ps-hierarchy-max-surface-variance` (\*) (default: `1000`)
:   The maximum surface variation of a cluster.

### WLOP (`ps-wlop`)

Weighted Locally Optimal Projection: produces a denoised, evenly distributed subset of the points.

`--tc-ps-wlop-retain-percent` (\*) (default: `1`)
:   The percentage `[0..100]` of points to keep. A single value is **doubled** at each zoom.

`--tc-ps-wlop-radius` (\*) (default: `10000`)
:   The radius of the neighborhood used to compute the projection.

`--tc-ps-wlop-iter-number` (default: `35`)
:   The number of iterations.

### Grid (`ps-grid`)

Overlays a 3D grid of cells on the points, and keeps one point per occupied cell.

`--tc-ps-grid-cell-size` (\*) (default: `10000`)
:   The size of the cells.

### Random (`ps-random`)

Removes a random subset of the points.

`--tc-ps-random-percent` (\*) (default: `90`)
:   The percentage `[0..100]` of points to remove. A single value is halved at each zoom, so give the value twice
    (e.g., `--tc-ps-random-percent 90 90`) to remove the same percentage at every zoom.

## Units

The point set algorithms work in Earth-centered, Earth-fixed (ECEF) coordinates, so the distances and sizes above
(`border-max-error`, `wlop-radius`, `grid-cell-size`...) are in **meters** at zoom 0. They are then scaled for the
deeper zooms as described in [parameters per zoom](../parameters-per-zoom.md).
