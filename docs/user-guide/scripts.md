# Helper scripts

The `scripts` folder contains two optional Python 3 scripts. They only use the Python standard library. In the
Docker image, they are installed in the `PATH`.

## `simple_tiles_server.py`

A minimal HTTP server, for **testing**, that serves a folder of quantized-mesh tiles with the headers Cesium needs:

- CORS headers, so that a viewer loaded from another origin can fetch the tiles;
- `Content-Encoding: gzip` and `Content-Type: application/vnd.quantized-mesh` for the `.terrain` files;
- `Cache-Control: no-cache`, so that regenerated tiles show up without clearing the browser's cache.

It serves several requests at a time, since Cesium requests many tiles at once.

```bash
simple_tiles_server.py [-p PORT] [folder]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `folder` | `./` | The folder to serve (the output folder of `qm_tiler`). |
| `-p PORT` | `8000` | The port the server listens on. |

The tiles are then available at `http://localhost:<PORT>/`. For production, see
[Serving the tiles in production](cesium.md#serving-the-tiles-in-production).

## `create_layer_file.py`

`qm_tiler` writes `layer.json` itself, so you **normally do not need this script**. Use it for tilesets that have no
`layer.json`, for example those created with versions older than v0.3, or after copying tiles between folders by hand.

The script scans the folder and builds `layer.json` from the tiles it finds:

```bash
create_layer_file.py [--no-normals] tms_dir
```

| Argument | Description |
|----------|-------------|
| `tms_dir` | The root of the tileset. `layer.json` is written there. |
| `--no-normals` | Does not declare the `octvertexnormals` extension. Use it if the tiles were created with `qm_tiler --no-normals`, or with a version older than v0.3, whose normals were not valid. |

The resulting file only differs from the one written by `qm_tiler` in `bounds`. The script does not know the extent
of the data, so it uses the outer edges of the tiles at the deepest zoom.
