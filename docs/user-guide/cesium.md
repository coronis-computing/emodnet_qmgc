# Visualizing in Cesium

The output folder of `qm_tiler` is a complete Cesium terrain tileset. To use it, serve the folder over HTTP and point
a `CesiumTerrainProvider` to it.

## The `layer.json` file

Cesium reads `layer.json` first, to learn the format of the tiles and which tiles exist. `qm_tiler` writes it at the
end of each run:

```json
{
    "tilejson": "2.1.0",
    "format": "quantized-mesh-1.0",
    "version": "1.2.0",
    "scheme": "tms",
    "tiles": ["{z}/{x}/{y}.terrain?v={version}"],
    "projection": "EPSG:4326",
    "minzoom": 0,
    "maxzoom": 10,
    "bounds": [-16.2625, 33.7375, -6.3625, 43.1375],
    "available": [
        [{"startX": 0, "startY": 0, "endX": 0, "endY": 0}],
        "..."
    ],
    "extensions": ["octvertexnormals"]
}
```

- `available` lists, for each zoom, the range of tiles created. Cesium indexes this list by zoom level, so it
  always starts at zoom 0, with empty entries for the zooms that were not built.
- `extensions` declares the per-vertex normals. Cesium only requests and decodes them if they are listed here. It is
  left out when the tiles are created with `--no-normals`.
- If the output folder already had a `layer.json`, the zooms it listed that were not built in this run are kept, and
  `bounds` grows to cover both.

For tilesets created with older versions, which did not write `layer.json`, use
[`create_layer_file.py`](scripts.md#create_layer_filepy).

## Serving the tiles for testing

Two HTTP headers are required:

- **CORS** (`Access-Control-Allow-Origin`), since the viewer is usually loaded from another origin;
- **`Content-Encoding: gzip`** for the `.terrain` files, which `qm_tiler` stores gzip-compressed. The browser then
  decompresses them transparently.

The bundled [`simple_tiles_server.py`](scripts.md#simple_tiles_serverpy) sets both:

```bash
simple_tiles_server.py -p 8000 tiles
```

## Serving the tiles in production

Any static web server works, as long as it sets the same headers. For example, with **nginx**:

```nginx
server {
    # ...
    root /srv/www;  # the output folder of qm_tiler is /srv/www/terrain

    location /terrain/ {
        add_header Access-Control-Allow-Origin * always;

        location ~ \.terrain$ {
            add_header Access-Control-Allow-Origin * always;
            add_header Content-Encoding gzip always;
            default_type application/vnd.quantized-mesh;
            gzip off;  # the files are already compressed
        }
    }
}
```

The terrain URL for Cesium is then `https://<your-server>/terrain/`.

With object storage (Amazon S3, Google Cloud Storage, Azure Blob...), upload the `.terrain` files with the metadata
`Content-Encoding: gzip` and `Content-Type: application/vnd.quantized-mesh`, and enable CORS on the bucket.

!!! warning "Do not compress the tiles twice"
    The `.terrain` files are already gzip-compressed. Make sure that the server does not compress them again, and
    that it does not decompress them before sending them without removing the `Content-Encoding` header.

## Client-side setup

```js
const terrainProvider = await Cesium.CesiumTerrainProvider.fromUrl("http://localhost:8000/", {
  requestVertexNormals: true,
});
const viewer = new Cesium.Viewer("cesiumContainer", { terrainProvider });
viewer.scene.globe.enableLighting = true;
```

- `requestVertexNormals: true` makes Cesium request the normals. Without it, Cesium does not shade the terrain,
  even though the tiles contain the normals.
- `enableLighting` turns on the shading of the globe by the sun.
- To exaggerate the relief, set `viewer.scene.verticalExaggeration` (CesiumJS 1.116 or later). Alternatively, scale
  the heights when creating the tiles with `--above-sea-level-scale-factor` / `--below-sea-level-scale-factor`.

## A complete viewer

The following page needs no Cesium ion account. It uses the imagery bundled with CesiumJS.
[Download it](../assets/viewer.html), adapt the URL, and open it in a browser:

```html title="viewer.html"
--8<-- "docs/assets/viewer.html"
```
