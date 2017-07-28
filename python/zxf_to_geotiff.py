#!/usr/bin/python

import argparse
import sys
from PIL import Image
from xml.dom import minidom
import glob
import os
from osgeo import osr
import re


def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(l, key = alphanum_key)


def main():
    parser = argparse.ArgumentParser(description="Converts a ZXF (i.e., MosaicViewer) into a geotiff")
    parser.add_argument('-i', dest='input_zxf', type=str, help="The input ZXF file")
    parser.add_argument('-if', dest='input_folder', type=str, default='./', help="The input folder containing the images (mosaic tiles) referenced by the ZXF")
    parser.add_argument('-of', dest='output_folder', type=str, default='./out', help="The output folder where georeferenced tiffs will be stored")
    parser.add_argument('--utm_zone', dest='utm_zone', type=int, help="The UTM zone where the mosaic is referenced (not stored in ZXF)")
    parser.add_argument('--utm_north', dest='utm_north', type=bool, default=True, help="North flag for UTM zone")
    parser.add_argument('--vrt_file', dest='vrt_file', type=str, default="", help="If specified, generates a VRT file joining all the geotiffs into a single dataset. This file will be stored in the output folder.")
    param = parser.parse_args()

    # Parse the ZXF file (an XML)
    xmldoc = minidom.parse(param.input_zxf)
    itemlist = xmldoc.getElementsByTagName("multiImageDef")
    mosaic_width = int( itemlist[0].attributes['width'].value )
    mosaic_height = int(itemlist[0].attributes['height'].value)
    itemlist = xmldoc.getElementsByTagName("tileDim")
    x_dim = int(itemlist[0].attributes['xDim'].value)
    itemlist = xmldoc.getElementsByTagName("MOSAIC_ORIGIN")
    mo_x = float(itemlist[0].attributes['x'].value)
    mo_y = float(itemlist[0].attributes['y'].value)
    itemlist = xmldoc.getElementsByTagName("MOSAIC_RESOLUTION")
    res = float(itemlist[0].attributes['resolution'].value)
    itemlist = xmldoc.getElementsByTagName("zoom")
    # Find the zoom scale 1
    for item in itemlist:
        if int(item.attributes['scale'].value) == 1:
            break
    base_name = item.firstChild.nodeValue

    # Georeference each tiff image
    files = glob.glob(param.input_folder+"*.tif")
    #files.sort() # This does not work! since _1 _2 _11 sorts to _1 _11 _2
    files = natural_sort(files)
    print files

    if not os.access(param.output_folder, os.F_OK):
        os.mkdir(param.output_folder)

    # Set the spatial reference system
    srs = osr.SpatialReference()
    srs.SetWellKnownGeogCS("WGS84")
    srs.SetUTM(param.utm_zone, param.utm_north)

    cur_x = 0
    cur_y = 0
    for file in files:
        file_base = os.path.basename(file)
        file_stem = os.path.splitext(file_base)[0]

        # Read the image
        im = Image.open(file)
        # Use GDAL (gdal_translate) to georeference the stored image
        ul_x = mo_x + (cur_x*res)
        ul_y = mo_y - (cur_y*res)
        lr_x = ul_x + (im.size[0]*res)
        lr_y = ul_y - (im.size[1]*res)

        cmd = "gdal_translate -of GTiff -a_ullr " + str(ul_x) + " " + str(ul_y) + " " + str(lr_x) + " " + str(lr_y) + " -a_srs " + srs.ExportToWkt() + " " + file_base + " " + param.output_folder+"/"+file_stem+"_geo.tif"
        print cmd
        os.system(cmd)

        if cur_x + x_dim >= mosaic_width:
            cur_x = 0
            cur_y = cur_y + im.size[1]
        else:
            cur_x = cur_x + im.size[0]

    if param.vrt_file:
        print "VRT file creation is not working yet, do it manually!"
        #cmd = "gdalbuildvrt " + param.vrt_file + " ".join(file)
        #print cmd
        #os.system(cmd)



if __name__ == '__main__':
    main()


