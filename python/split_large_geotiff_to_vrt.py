#!/usr/bin/python

import argparse
import os
import subprocess
import gdal


def main():
    parser = argparse.ArgumentParser(description="Converts EMODnet ESRI Ascii files into geotiffs and copies the georeferencing from the RGB geotiffs")
    parser.add_argument("input_tif", action="store", type=str, help="The TIFF file to split")
    parser.add_argument('--tile_size_x', dest='tile_size_x', type=int, default=1024, help="The size of the tiles' size in X")
    parser.add_argument('--tile_size_y', dest='tile_size_y', type=int, default=1024, help="The size of the tiles' size in Y")
    parser.add_argument('-o', dest='output_dir', type=str, default='./out', help="The output folder where the tiles and the VRT file will be created")
    param = parser.parse_args()

    # File name base
    file_base = os.path.basename(param.input_tif)
    file_stem = os.path.splitext(file_base)[0]

    # Create the output dir if not existing
    if not os.access(param.output_dir, os.F_OK):
        os.mkdir(param.output_dir)

    # Extract the WKT from the geotiff
    wkt_str = subprocess.check_output(["gdalsrsinfo", "-o", "wkt", param.input_tif])
    wkt_str = wkt_str.rstrip('\n')  # Remove newline chars

    # Open a GDAL dataset with the geotiff as input
    ds = gdal.Open(param.input_tif)
    band = ds.GetRasterBand(1)
    xsize = band.XSize
    ysize = band.YSize

    # Split the image
    for i in range(0, xsize, param.tile_size_x):
        for j in range(0, ysize, param.tile_size_y):
            cmd = "gdal_translate -of GTIFF -srcwin " + str(i)+ ", " + str(j) + ", " + str(param.tile_size_x) + ", " + str(param.tile_size_y) + " " + param.input_tif + " " + param.output_dir + "/" + file_stem + "_x" + str(i) + "_y" + str(j) + ".tif"
            os.system(cmd)

    # Create a VRT file joining the resulting geotiffs, to ease use in GDAL
    cmd = "gdalbuildvrt " + param.output_dir + "/" + file_stem + ".vrt" + " " + param.output_dir + "/*.tif"
    os.system(cmd)



if __name__ == '__main__':
    main()
