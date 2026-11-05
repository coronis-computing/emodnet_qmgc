# Edge collapse (Lindstrom-Turk)

`--tc-strategy lt` creates the TIN by simplifying the full-resolution mesh of the tile with the Lindstrom-Turk edge
collapse algorithm:

1. P. Lindstrom and G. Turk. *Fast and memory efficient polygonal simplification.* In IEEE Visualization, pages
   279–286, 1998.
2. P. Lindstrom and G. Turk. *Evaluation of memoryless simplification.* IEEE Transactions on Visualization and
   Computer Graphics, 5(2):98–115, 1999.

We adapted the implementation in [CGAL](https://doc.cgal.org/latest/Surface_mesh_simplification/index.html) to
preserve the borders of the tiles.

## How it works

The method starts by triangulating all the samples of the tile. Then, at each step, it **collapses** one edge of the
mesh into a single vertex. The method relies on two concepts:

- the **cost** of collapsing an edge;
- the **placement** of the vertex that replaces the collapsed edge.

The edges are kept in a priority queue sorted by cost. At each step, the edge with the smallest cost is collapsed, the
connectivity around it is updated, and the costs of the affected edges are recomputed. The process ends when the mesh
has the desired number of edges.

The vertices on the borders of the tile are kept in place, so that the tile matches its neighbors.

## Parameters

`--tc-lt-stop-edges-count` (\*) (default: `500`)
:   The simplification stops when the number of edges in the mesh is below this value. It directly controls the size
    of the tiles. Unlike other per-zoom parameters, a single value is **not** scaled: it is used for every zoom. Give
    several values to use a different count per zoom.

`--tc-lt-weight-volume` (default: `0.5`)
`--tc-lt-weight-boundary` (default: `0.5`)
`--tc-lt-weight-shape` (default: `1e-10`)
:   The weights of the volume, boundary and shape optimization terms in the cost and placement functions. The default
    values give good results in most cases. See the original references for their meaning.

## Example

```bash
qm_tiler -i dem.tif -o tiles --tc-strategy lt --tc-lt-stop-edges-count 1000
```
