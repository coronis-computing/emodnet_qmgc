#!/usr/bin/python

import argparse
import os
import sys

class my_colors:
    Red = '\033[31m'
    Green = '\033[32m'
    Yellow = '\033[33m'
    Blue = '\033[34m'
    Magenta = '\033[35m'
    Cyan = '\033[36m'
    LightGray = '\033[37m'
    DarkGray = '\033[90m'
    LightRed = '\033[91m'
    LightGreen = '\033[92m'
    LightYellow = '\033[93m'
    LightBlue = '\033[94m'
    LightMagenta = '\033[95m'
    LightCyan = '\033[96m'
    ENDC = '\033[0m'

def print_step(step_name):
    header = "*** " + step_name + " ***"
    print "\n" + my_colors.Cyan + header + my_colors.ENDC

def print_cmd(cmd, fancy_output):
    if fancy_output:
        print my_colors.Green + cmd + my_colors.ENDC
    else:
        print cmd

def run_cmd(cmd):
    # Run the command
    failure = os.system(cmd)
    # Check for failure
    if failure:
        print '[ERROR] Execution of command: "%s" FAILED!\nAborting...' % cmd
        return False
    return True



def main():
    parser = argparse.ArgumentParser(description="Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format.\n"
                                                 "This script avoids the ""integer overflow"" problem at lower zoom levels for large datasets.\n"
                                                 "The qm_tiler app, as ctb-tile did, resamples data from the source dataset, and this may lead to integer overflow issues when resampling the tiles at low zoom levels, as these will be very large")
    parser.add_argument("--input", "-i", dest="input_file", action="store", type=str, help="The GDAL raster file to tile")
    parser.add_argument("--output-dir", "-o", dest="output_dir", type=str, default="./terrain_tiles_qm", help="The output folder where the Quantized Mesh tiles will be created")
    parser.add_argument("--output-dir-tiff", "-t", dest="output_dir_tiff", type=str, default="./terrain_tiles_tiff",
                        help="The output folder where the (temporary) tiff tiles will be created")
    parser.add_argument("--start-zoom", "-s", dest="start_zoom", type=int, default=10, help="The zoom level to start at. This should be greater than the end zoom level")
    parser.add_argument("--end-zoom", "-e", dest="end_zoom", type=int, default=0,
                         help="The zoom level to end at. This should be less than the start zoom level and >= 0. If smaller than zero, defaults to max_zoom")
    parser.add_argument("--start-zoom-tiff", dest="start_zoom_tiff", type=int, default=0,
                         help="The zoom level at which the tiff pyramids will be used to aid the quantized mesh generation. All the zooms from here to end-zoom will use the tiff tiles from the previous zoom as base raster")
    parser.add_argument("--end-zoom-tiff", dest="end_zoom_tiff", type=int, default=0,
                        help="The zoom level at which we stop building tiff zooms. All the zooms from here to end-zoom will use this tiff zoom as base raster")
    parser.add_argument("--params", "-p", dest="qm_tiler_params", type=str, default="",
                        help="A string containing all the parameters to be passed to qm_tiler (put all them under \"\"). Remember that you can use an external config file to pass the parameters. In this later case, this parameter would be something like \"-c params.cfg\". Note that you should skip the -i, -o, -s and -e parameters in qm_tiler, because they must be specified using the corresponding parameters of this script.")
    parser.add_argument("--fancy-output", "-f", dest="fancy_output", action='store_true', help="Activates fancy output of the progress, where the different steps of the script are shown in blue and the commands run in green")
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

    if param.end_zoom_tiff > param.start_zoom:
        print "[ERROR] If you do not want to use the intermediate tif tiles (--end_zoom_tiff/--end_zoom > --start_zoom), then you should probably use the qm_tiler app directly"
        return

    if param.start_zoom_tiff == 0 & param.start_zoom > 0:
        param.start_zoom_tiff = param.start_zoom

    if param.start_zoom_tiff < 0:
        print "[WARNING] Invalid start zoom level for tiffs, defaulting to start-zoom"
        param.start_zoom_tif = param.start_zoom

    if param.start_zoom_tiff > param.start_zoom:
        print "[ERROR] Invalid end zoom level for tiffs, it should be smaller or equal to the start-zoom-tiff"
        return

    if param.start_zoom_tiff < param.end_zoom:
        print "[ERROR] If you do not want to use the intermediate tif tiles (--start_zoom_tiff < --end_zoom), then you should probably use the qm_tiler app directly"
        return

    if "-i " in param.qm_tiler_params or "--input " in param.qm_tiler_params or "-o " in param.qm_tiler_params or "--output-dir " in param.qm_tiler_params or "-e " in param.qm_tiler_params or "--end-zoom " in param.qm_tiler_params or "-s " in param.qm_tiler_params or "--start-zoom " in param.qm_tiler_params:
        print "[ERROR] You should not specify the -i, -o, -s and/or -e parameters in qm_tiler's parameters (--param/-p). They must be specified using the corresponding parameters of this script."
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
    further_options = " " + param.qm_tiler_params + " "

    # Create the tiles of the terrain for the deepest zoom levels (without intermediate tiffs)
    if param.fancy_output: print_step("Create zooms without intermediate base Tiffs")
    if param.start_zoom > param.start_zoom_tiff:
        cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(param.start_zoom) + " --end-zoom " + str(param.start_zoom_tiff+1) + " " + further_options + param.input_file
        print_cmd(cmd, param.fancy_output)
        all_ok = run_cmd(cmd)
        if not all_ok:
            sys.exit(1)

    # Create the tiles of the terrain for the selected zooms using a base of tiff tiles of a deeper zoom in the pyramid
    if param.fancy_output: print_step("Create zooms using a base of Tiff tiles of a deeper zoom in the pyramid")
    for i in xrange(param.start_zoom_tiff+1, param.end_zoom_tiff, -1):
        # Create the GDAL tileset for this zoom level
        cmd = "ctb-tile --output-format GTiff --output-dir " + param.output_dir_tiff + " --start-zoom " + str(i) + " --end-zoom " + str(i) + " " + param.input_file
        print_cmd(cmd, param.fancy_output)
        all_ok = run_cmd(cmd)
        if not all_ok:
            sys.exit(1)

        # Build the VRT from this level
        cmd = "gdalbuildvrt " + param.output_dir_tiff + "/" + "zoom" + str(i) + ".vrt " + param.output_dir_tiff + "/" + str(i) + "/*/*.tif"
        print_cmd(cmd, param.fancy_output)
        all_ok = run_cmd(cmd)
        if not all_ok:
            sys.exit(1)

        # Construct the quantized mesh terrain tiles from the VRT of the previous zoom level
        cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(i-1) + " --end-zoom " + str(i-1) + " " + further_options +  param.output_dir_tiff + "/zoom" + str(i) + ".vrt"
        print_cmd(cmd, param.fancy_output)
        all_ok = run_cmd(cmd)
        if not all_ok:
            sys.exit(1)

    # Create the rest of zooms from the remaining zooms from the last tiff base
    if param.fancy_output: print_step("Create zooms using the last base Tiff")
    cmd = "qm_tiler --output-dir " + param.output_dir + " --start-zoom " + str(param.end_zoom_tiff-1) + " --end-zoom " + str(param.end_zoom) + " " + further_options + param.output_dir_tiff + "/zoom" + str(param.end_zoom_tiff+1) + ".vrt"
    print_cmd(cmd, param.fancy_output)
    all_ok = run_cmd(cmd)
    if not all_ok:
        sys.exit(1)

if __name__ == '__main__':
    main()
