// Copyright (c) 2019 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is subject to the terms and conditions defined in the 
// 'LICENSE.txt' file, which is part of this source code package.
//
// Author: Ricard Campos (ricard.campos@coronis.es)

#include <iostream>
#include <boost/program_options.hpp>
#include <ctb.hpp>

using namespace std;
namespace po = boost::program_options;

int main ( int argc, char **argv) {
    // Parse input parameters
    int x, y, z;
    po::options_description options("Get the Global Geodetic coordinates (lat/lon) for the bounds of a tile given its (zoom, x, y) coordinates");
    bool displayTileInfo, displayTileStats, displayZoomStats;
    options.add_options()
            ("help,h", "Produce help message")
            ("zoom,z", po::value<int>(&z),
             "Zoom level")
            ("x-coord,x", po::value<int>(&x),
             "X coordinate within zoom")
            ("y-coord,y", po::value<int>(&y),
             "Y coordinate within zoom");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    ctb::GlobalGeodetic profile;
    ctb::TileCoordinate coord(z, x, y);
    ctb::CRSBounds tileBounds = profile.tileBounds(coord);
    cout << "Tile Bounds:" << endl;
    cout << "minX = " << tileBounds.getMinX() << endl;
    cout << "maxX = " << tileBounds.getMaxX() << endl;
    cout << "minY = " << tileBounds.getMinY() << endl;
    cout << "maxY = " << tileBounds.getMaxY() << endl;

    return EXIT_SUCCESS;
}