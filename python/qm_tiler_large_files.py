#!/usr/bin/python

import argparse
import os



def main():
    parser = argparse.ArgumentParser(description="Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format.\n"
                                                 "This script avoids the ""integer overflow"" problem at lower zoom levels for large datasets.\n"
                                                 "The qm_tiler app, as ctb-tile did, resamples data from the source dataset, and this may lead to integer overflow issues when resampling the tiles at low zoom levels, as these will be very large")
    parser.add_argument("input_file", action="store", type=str, help="The GDAL raster file to tile")
    parser.add_argument("--output-dir", "-o", dest="output_dir", type=str, default="./terrain_tiles_qm", help="The output folder where the Quantized Mesh tiles will be created")
    parser.add_argument("--output-dir-tiff", "-t", dest="output_dir_tiff", type=str, default="./terrain_tiles_tiff",
                        help="The output folder where the (temporary) tiff tiles will be created")
    parser.add_argument("--start-zoom", "-s", dest="start_zoom", type=int, default=10, help="The zoom level to start at. This should be greater than the end zoom level")
    parser.add_argument("--end-zoom", "-e", dest="end_zoom", type=int, default=0,
                        help="The zoom level to end at. This should be less than the start zoom level and >= 0. If smaller than zero, defaults to max_zoom")
    parser.add_argument("--end-zoom-tiff", dest="end_zoom_tiff", type=int, default=0,
                        help="The zoom level at which the tiff pyramids will be used to aid the quantized mesh generation. All the zooms from here to end-zoom will use this zoom as base raster")
    parser.add_argument("--bathymetry", "-b", dest="bathymetry_flag", action='store_true', default=False,
                        help="Switch to consider the input DEM as containing depths instead of elevations")
    parser.add_argument("--simp-stop-edges-count", dest="simp_stop_edges_count", type=int, default=500,
                        help="Simplification stops when the number of edges is below this value.")
    parser.add_argument("--samples-per-tile", dest="samples_per_tile", type=int, default=256,
                        help="Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that a smaller sampling provides a coarser base mesh that will be easier to simplify.")
    parser.add_argument("--simp-weight-volume", dest="simp_weight_volume", type=float, default=0.5,
                        help="Simplification volume weight (Lindstrom-Turk cost function, see original reference).")
    parser.add_argument("--simp-weight-boundary", dest="simp_weight_boundary", type=float, default=0.5,
                        help="Simplification boundary weight (Lindstrom-Turk cost function, see original reference).")
    parser.add_argument("--simp-weight-shape", dest="simp_weight_shape", type=float, default=1e-10,
                        help="Simplification shape weight (Lindstrom-Turk cost function, see original reference).")

    param = parser.parse_args()

    # Check input parameters
    if param.start_zoom < 0:
        print "[WARNING] Invalid start zoom level, defaulting to 0"
        param.start_zoom = 0

    if param.end_zoom < 0:
        print "[WARNING] Invalid end zoom level, defaulting to 0"
        param.end_zoom = 0

    if param.end_zoom_tiff == 0 & param.end_zoom > 0:
        param.end_zoom_tif = param.end_zoom

    if param.end_zoom_tiff < 0:
        print "[WARNING] Invalid end zoom level for tiffs, defaulting to end-zoom"
        param.end_zoom_tif = param.end_zoom

    if param.end_zoom_tiff < param.end_zoom:
        print "[ERROR] Invalid end zoom level for tiffs, it should be larger than the end-zoom-tiff"
        return

    if param.end_zoom_tiff >= param.start_zoom:
        print "[ERROR] If you do not want to use the intermediate tif tiles (--end_zoom_tiff/--end_zoom >= --start_zoom), then you should probably use the qm_tiler app directly"
        return

    # File name base
    file_base = os.path.basename(param.input_file)
    file_stem = os.path.splitext(file_base)[0]

    # Create the output dirs if not existing
    if not os.access(param.output_dir, os.F_OK):
        os.mkdir(param.output_dir)
    if not os.access(param.output_dir_tiff, os.F_OK):
        os.mkdir(param.output_dir_tiff)

    # Further options to be passed to qm_tiler
    further_options = "--samples-per-tile " + str(param.samples_per_tile) + " --simp-weight-volume " + str(param.simp_weight_volume) + " --simp-weight-boundary " + str(param.simp_weight_boundary) + " --simp-weight-shape " + str(param.simp_weight_shape) + " "
    if param.bathymetry_flag:
        further_options = further_options + "-b "

    # Create the tiles of the terrain for the highest zoom level
    cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(param.start_zoom) + " --end-zoom " + str(param.start_zoom) + " " + further_options + param.input_file
    print cmd
    os.system(cmd)

    # Create the GDAL tileset for this zoom level
    cmd = "ctb-tile --output-format GTiff --output-dir " + param.output_dir_tiff + " --start-zoom " + str(param.start_zoom) + " --end-zoom " + str(param.start_zoom) + " " + param.input_file
    print cmd
    os.system(cmd)

    # Build the VRT from this level
    cmd = "gdalbuildvrt " + param.output_dir_tiff + "/" + "zoom" + str(param.start_zoom) + ".vrt " + param.output_dir_tiff + "/" + str(param.start_zoom) + "/*/*.tif"
    print cmd
    os.system(cmd)

    # Construct the quantized mesh terrain tiles from the VRT of the previous zoom level
    cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(param.start_zoom-1) + " --end-zoom " + str(param.start_zoom-1) + " " + further_options +  param.output_dir_tiff + "/zoom" + str(param.start_zoom) + ".vrt"
    print cmd
    os.system(cmd)

    # Create the zooms with a base of tiffs
    for i in xrange(param.start_zoom-1, param.end_zoom_tiff-1, -1):
        # Create the tiff tiles of the current zoom level from the previous zoom's VRT
        cmd = "ctb-tile --output-format GTiff --output-dir " + param.output_dir_tiff + " --start-zoom " + str(i) + " --end-zoom " + str(i) + " " + param.output_dir_tiff + "/zoom" + str(i+1) + ".vrt"
        print cmd
        os.system(cmd)

        # Create the VRT for this level
        cmd = "gdalbuildvrt " + param.output_dir_tiff + "/" + "zoom" + str(i) + ".vrt " + param.output_dir_tiff + "/" + str(i) + "/*/*.tif"
        print cmd
        os.system(cmd)

        # Construct the quantized mesh terrain tiles from the VRT of the previous zoom level
        cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(i) + " --end-zoom " + str(i) + " " + further_options + param.output_dir_tiff + "/zoom" + str(i+1) + ".vrt"
        print cmd
        os.system(cmd)

    # Create the rest of zooms from the remaining tiffs
    for i in xrange(param.end_zoom_tiff-1, param.end_zoom-1, -1):
        # Construct the quantized mesh terrain tiles from the VRT of the previous zoom level
        cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(i) + " --end-zoom " + str(i) + " " + further_options + param.output_dir_tiff + "/zoom" + str(param.end_zoom_tiff) + ".vrt"
        print cmd
        os.system(cmd)

if __name__ == '__main__':
    main()