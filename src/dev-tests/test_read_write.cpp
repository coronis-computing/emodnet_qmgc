// Copyright (c) 2018 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is part of EMODnet Quantized Mesh Generator for Cesium.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Author: Ricard Campos (ricardcd@gmail.com)

/**
 * @file
 * @brief Reads a quantized mesh terrain file, writes it to disk, and reads it back.
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// Boost
#include <boost/program_options.hpp>
// Std
#include <iostream>
// Project-specific
#include "quantized_mesh_tile.h"
// Cesium-terrain-builder
#include <ctb.hpp>

using namespace std ;
namespace po = boost::program_options ;



int main ( int argc, char **argv)
{
    // Parse input parameters
    std::string inputFile, outputFile ;
    int x = -1, y = -1, z = -1 ;
    po::options_description options("Reads a quantized mesh terrain file, writes it to disk, and reads it back.\n"
                                    "The contents are shown in plain text on the screen for visual comparison") ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
            ( "output,o", po::value<std::string>(&outputFile)->default_value("./tmp.terrain"), "Output terrain file to write. The input file will be read and written back using library functions. Results should be the same!" )
            ( "tileX,x", po::value<int>(&x), "Tile X" )
            ( "tileY,y", po::value<int>(&y), "Tile Y" )
            ( "tileZ,z", po::value<int>(&z), "Tile Zoom" )
            ;

    po::variables_map vm ;
    po::store( po::parse_command_line(argc, argv, options), vm ) ;
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Read the quantized mesh tile
    cout << "- Reading the file " << inputFile << endl ;
    const ctb::TileCoordinate coord(z, x, y);
    QuantizedMeshTile qmt(coord);
    if ( !qmt.readFile(inputFile) ) {
        cerr << "[ERROR] Cannot read the input file" << endl ;
        return -1 ;
    }

    // Print information on screen
    qmt.print() ;

    // Write it to file
    cout << "- Writing the file " << outputFile << endl ;
    if ( !qmt.writeFile( outputFile ) ) {
        cerr << "[ERROR] Cannot write the output file" << endl ;
        return -1 ;
    }

    // Read the written file back and check if it is the same as input
    cout << "- Reading the file " << outputFile << " back" << endl ;
    QuantizedMeshTile qmt2(coord);
    if ( !qmt2.readFile( outputFile ) ) {
        cerr << "[ERROR] Cannot read the output file" << endl ;
        return -1 ;
    }

    qmt2.print() ;

    return 1 ;
}