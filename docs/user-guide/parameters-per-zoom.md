# Setting parameters per zoom

The same TIN creation method is used for every zoom level of the pyramid, but the right value of its parameters
usually depends on the zoom. A tile at zoom 0 covers 180° × 180°, while a tile at zoom 10 covers about 0.18° × 0.18°.
An error tolerance of 100 m is very tight for the former and very loose for the latter.

The options marked with (\*) can therefore take a different value for each zoom. There are two ways of setting them.

## A single value: scaled automatically

If you give **a single value**, it is the value for the **root of the pyramid (zoom 0)**. The value for every other
zoom is computed from it, using the quadtree structure of the pyramid: each zoom halves the size of the tiles.

- For parameters whose scale must **decrease** with depth (tolerances, sizes, distances):

    \[ p_z = \frac{p_0}{2^z} \]

- For parameters whose scale must **increase** with depth:

    \[ p_z = p_0 \cdot 2^z \]

For example, `--tc-greedy-error-tol 5120` gives a tolerance of 5120 m at zoom 0, 2560 m at zoom 1, ... and 5 m at
zoom 10.

To get the value for zoom 0 from the value \(p_z\) you want at zoom \(z\), invert the formula:
\(p_0 = p_z \cdot 2^z\).

## Several values: one per zoom

If you give **several values**, the \(i\)-th value is used for zoom \(i\). Zooms deeper than the last value given use
the last value. No scaling is applied.

On the command line, give all the values after the option:

```bash
# 64 cells at zoom 0, 32 at zoom 1, 16 at zoom 2, and 0 (no grid) from zoom 3 on
qm_tiler -i dem.tif --tc-greedy-init-grid-size 64 32 16 0
```

In a [configuration file](general-parameters.md#configuration-file), repeat the option, one line per value:

```ini
tc-greedy-init-grid-size = 64
tc-greedy-init-grid-size = 32
tc-greedy-init-grid-size = 16
tc-greedy-init-grid-size = 0
```

!!! note
    The values are always indexed from zoom 0, whatever the zoom range you build. To set a value only for zooms 8
    and deeper, you still need to give the values for zooms 0 to 7.

## How each parameter scales

When a single value is given:

| Parameter | Scaling of a single value |
|-----------|---------------------------|
| `--tc-greedy-error-tol` | \(p_0 / 2^z\) |
| `--tc-greedy-init-grid-size` | \(p_0 / 2^z\), rounded down to an integer |
| `--tc-lt-stop-edges-count` | **Not scaled:** the same value is used at every zoom |
| `--tc-ps-border-max-error` | \(p_0 / 2^z\) |
| `--tc-ps-border-max-length-xy-percent` | \(p_0 / 2^z\) |
| `--tc-ps-hierarchy-cluster-size` | \(p_0 / 2^z\) |
| `--tc-ps-hierarchy-max-surface-variance` | \(p_0 / 2^z\) |
| `--tc-ps-wlop-retain-percent` | \(p_0 \cdot 2^z\) |
| `--tc-ps-wlop-radius` | \(p_0 / 2^z\) |
| `--tc-ps-grid-cell-size` | \(p_0 / 2^z\) |
| `--tc-ps-random-percent` | \(p_0 / 2^z\) |

!!! warning "Parameters that are not scale dependent"
    Some parameters are counts or percentages, which do not depend on the size of the tile. If you give a single
    value for these, it is still halved at every zoom, which is rarely what you want. For example,
    `--tc-greedy-init-grid-size 8` gives a grid of 8 cells at zoom 0, 4 at zoom 1, 2 at zoom 2, 1 at zoom 3, and none
    from zoom 4 on. Likewise, `--tc-ps-random-percent 90` removes 90% of the points at zoom 0 but only 45% at
    zoom 1.

    For these parameters, give one value per zoom instead. A single repeated value works too: `--tc-ps-random-percent
    90 90` keeps 90% at every zoom, because two values disable the scaling and the last one is reused for the deeper
    zooms.
