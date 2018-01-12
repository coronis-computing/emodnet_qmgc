#!/usr/bin/python

import argparse
import sys
from PIL import Image
from xml.dom import minidom
import glob
import os
from osgeo import osr
import subprocess
import re


def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)


def main():
    parser = argparse.ArgumentParser(description="Converts EMODnet ESRI Ascii files into geotiffs and copies the georeferencing from the RGB geotiffs")
    parser.add_argument('--input_esri_folder', dest='input_esri_folder', type=str, default='./', help="The input folder containing the ESRI Ascii files to convert")
    parser.add_argument('--input_geotiff_folder', dest='input_geotif_folder', type=str, help="The input folder containing the geotiff images from where we will extract the wkt")
    parser.add_argument('-of', dest='output_folder', type=str, default='./out', help="The output folder where georeferenced tiffs will be stored")
    parser.add_argument('--vrt_file', dest='vrt_file', type=str, default="", help="If specified, generates a VRT file joining all the geotiffs into a single dataset. This file will be stored in the output folder.")
    param = parser.parse_args()

    # Georeference each tiff image
    files = glob.glob(param.input_esri_folder+"/*.asc")
    print files

    if not os.access(param.output_folder, os.F_OK):
        os.mkdir(param.output_folder)

    output_files = []
    for file in files:
        file_base = os.path.basename(file)
        file_stem = os.path.splitext(file_base)[0]

        # Extract the WKT from the RGB geotiffs
        ref_geotif_file = param.input_geotif_folder + "/" + file_stem +"_rgb.tif"
        print ref_geotif_file
        wkt_str = subprocess.check_output(["gdalsrsinfo", "-o", "wkt", ref_geotif_file])
        wkt_str = wkt_str.rstrip('\n') # Remove newline chars

        output_file = file_stem+"_geo.tif"
        output_file_full_path = param.output_folder+"/"+output_file
        cmd = "gdal_translate -of GTiff -a_srs " + wkt_str + " " + file_base + " " + output_file_full_path
        print cmd
        os.system(cmd)

        output_files.append(output_file)

    if param.vrt_file:
        print "VRT file creation is not working yet, do it manually!"
        #os.system("cd " + param.output_folder)
        #cmd = "gdalbuildvrt " + param.vrt_file + " ".join(output_files)
        #print cmd
        #os.system(cmd)



if __name__ == '__main__':
    main()


