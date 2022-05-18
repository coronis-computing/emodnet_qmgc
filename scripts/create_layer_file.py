#!/usr/bin/python

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
    parser = argparse.ArgumentParser(description="Creates the layer.json file required by Cesium from a TMS folder structure (quantized-mesh format assumed)")
    parser.add_argument("tms_dir", action="store", type=str, help="The root of a TMS directory in Cesium''s quantized-mesh format. The generated layer.json file will be created here")

    param = parser.parse_args()

    # Base layer.json data
    data = {"tilejson": "2.1.0", "format": "quantized-mesh-1.0", "version": "1.2.0", "scheme": "tms", "tiles": ["{z}/{x}/{y}.terrain?v={version}"], "attribution": "Terrain Data,Author: Created using the EMODnet Quantized Mesh Generator for Cesium", "projection": "EPSG:4326"}

    # Get the zoom levels

    # List of folders in the base level
    list_zoom_dirs = get_folders(param.tms_dir)
    list_zoom_ints = map(int, list_zoom_dirs) # As integers

    # Extract the minimum and maximum zoom from the available dirs
    data["minzoom"] = min(list_zoom_ints)
    data["maxzoom"] = max(list_zoom_ints)

    # Get the limits for each zoom
    list_zoom_ints = sorted(list_zoom_ints)

    available = []
    for folder_x_int in list_zoom_ints:
        cur_limits = {}
        folder_x = str(folder_x_int)

        # --- Limits in X ---
        dir_cnt_X = get_folders(os.path.join(param.tms_dir, folder_x))
        dir_cnt_X_ints = map(int, dir_cnt_X)  # As integers
        cur_limits["startX"] = min(dir_cnt_X_ints)
        cur_limits["endX"] = max(dir_cnt_X_ints)

        # --- Limits in Y ---
        dir_cnt_Y = get_files(os.path.join(param.tms_dir, folder_x, dir_cnt_X[0])) # We use the first folder in X, they are all supposed to have the same number of tiles and with the same numbering
        dir_cnt_Y = [f.replace(".terrain", "") for f in dir_cnt_Y] # Remove the ".terrain" part of the files
        dir_cnt_Y_ints = map(int, dir_cnt_Y)  # As integers
        cur_limits["startY"] = min(dir_cnt_Y_ints)
        cur_limits["endY"] = max(dir_cnt_Y_ints)

        cur_limits_list = [cur_limits]
        available.append(cur_limits_list)

    # Compute the bounds of the map based on the coordinates of the tiles in the deeper zoom level
    deeper_limits = available[data["maxzoom"]][0]
    s, w, _, _ = TileLatLonBounds(deeper_limits["startX"], deeper_limits["startY"], data["maxzoom"])
    _, _, n, e = TileLatLonBounds(deeper_limits["endX"], deeper_limits["endY"], data["maxzoom"])

    data["bounds"] = [w, s, e, n]
    data["available"] = available

    # Write JSON file
    with open(param.tms_dir + "/layer.json", 'w') as outfile:
        json.dump(data, outfile, indent = 4)
