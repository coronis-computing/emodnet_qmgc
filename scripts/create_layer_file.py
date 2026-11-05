#!/usr/bin/env python3

import argparse
import json
import os


# Returns a list with the folders in a given directory
def get_folders(dir):
    dir_cnt = os.listdir(dir)
    return [name for name in dir_cnt if os.path.isdir(os.path.join(dir, name))]

# Returns a list with the files in a given directory
def get_files(dir):
    dir_cnt = os.listdir(dir)
    return [name for name in dir_cnt if os.path.isfile(os.path.join(dir, name))]

# DevNote: the following two functions come from gdal2tiles sources:
def TileBounds(tx, ty, zoom):
        "Returns bounds of the given tile"
        tileSize = 256
        res = 180.0 / tileSize / 2**zoom
        return (
            tx*tileSize*res - 180,
            ty*tileSize*res - 90,
            (tx+1)*tileSize*res - 180,
            (ty+1)*tileSize*res - 90
        )
        
def TileLatLonBounds(tx, ty, zoom):
    "Returns bounds of the given tile in the SWNE form"
    b = TileBounds(tx, ty, zoom)
    return (b[1],b[0],b[3],b[2])

# Main function
if __name__ == '__main__':
    #Parameters
    parser = argparse.ArgumentParser(description="Creates the layer.json file required by Cesium from the tiles found in a TMS folder structure (quantized-mesh format assumed). Note that qm_tiler already writes this file, so this script is only needed for existing tilesets, e.g., ones created with older versions. An existing layer.json file in the folder is overwritten.")
    parser.add_argument("tms_dir", action="store", type=str, help="The root of a TMS directory in Cesium''s quantized-mesh format. The generated layer.json file will be created here")

    parser.add_argument("--no-normals", dest="no_normals", action="store_true", help="Do not declare the oct-encoded per-vertex normals extension (use it when the tiles were created with qm_tiler --no-normals)")

    param = parser.parse_args()

    # Base layer.json data
    data = {"tilejson": "2.1.0", "format": "quantized-mesh-1.0", "version": "1.2.0", "scheme": "tms", "tiles": ["{z}/{x}/{y}.terrain?v={version}"], "attribution": "Created using the EMODnet Quantized Mesh Generator for Cesium", "projection": "EPSG:4326"}
    if not param.no_normals:
        # Note: Cesium only requests and decodes the normals if the extension is listed here
        data["extensions"] = ["octvertexnormals"]

    # Get the zoom levels

    # List of folders in the base level (only the numeric ones are zoom levels)
    # Note: lists rather than map() objects, which in Python 3 can only be consumed once
    list_zoom_ints = sorted(int(d) for d in get_folders(param.tms_dir) if d.isdigit())
    if not list_zoom_ints:
        raise SystemExit("[ERROR] No zoom level folders found in " + param.tms_dir)

    # Extract the minimum and maximum zoom from the available dirs
    data["minzoom"] = min(list_zoom_ints)
    data["maxzoom"] = max(list_zoom_ints)

    # Get the limits for each zoom
    # Note: Cesium indexes "available" by zoom level, so it must start at zoom 0
    # even if the pyramid does not. Levels without tiles get an empty list.
    available = [[] for _ in range(data["maxzoom"] + 1)]
    for zoom in list_zoom_ints:
        folder_zoom = os.path.join(param.tms_dir, str(zoom))

        # --- Limits in X ---
        dir_cnt_X = [d for d in get_folders(folder_zoom) if d.isdigit()]
        if not dir_cnt_X:
            continue
        dir_cnt_X_ints = [int(d) for d in dir_cnt_X]

        # --- Limits in Y ---
        # Gathered over all the X folders, rather than assuming they all contain the same tiles
        dir_cnt_Y_ints = []
        for folder_x in dir_cnt_X:
            files_Y = get_files(os.path.join(folder_zoom, folder_x))
            dir_cnt_Y_ints.extend(int(f[:-len(".terrain")]) for f in files_Y
                                  if f.endswith(".terrain") and f[:-len(".terrain")].isdigit())
        if not dir_cnt_Y_ints:
            continue

        cur_limits = {"startX": min(dir_cnt_X_ints), "endX": max(dir_cnt_X_ints),
                      "startY": min(dir_cnt_Y_ints), "endY": max(dir_cnt_Y_ints)}
        available[zoom] = [cur_limits]

    # Compute the bounds of the map based on the coordinates of the tiles in the deeper zoom level
    deeper_limits = available[data["maxzoom"]][0]
    s, w, _, _ = TileLatLonBounds(deeper_limits["startX"], deeper_limits["startY"], data["maxzoom"])
    _, _, n, e = TileLatLonBounds(deeper_limits["endX"], deeper_limits["endY"], data["maxzoom"])

    data["bounds"] = [w, s, e, n]
    data["available"] = available

    # Write JSON file
    with open(param.tms_dir + "/layer.json", 'w') as outfile:
        json.dump(data, outfile, indent = 4)
