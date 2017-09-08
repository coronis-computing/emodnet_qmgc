//
// Created by Ricard Campos
//

// Boost
#include <boost/program_options.hpp>
// Std
#include <iostream>
// Project-specific
#include "quantized_mesh_tile.h"

using namespace std ;
namespace po = boost::program_options ;

int main ( int argc, char **argv)
{
    // Parse input parameters
    std::string inputFile, outputFile ;
    int x = -1, y = -1, z = -1 ;
    po::options_description options("Reads a quantized mesh terrain file") ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file in heightmap-1.0 format" )
            ( "output,o", po::value<std::string>(&outputFile), "Output terrain file in quantized-mesh-1.0 format" )
            ( "tileX,x", po::value<int>(&x), "Tile X" )
            ( "tileY,y", po::value<int>(&y), "Tile Y" )
            ( "tileZ,z", po::value<int>(&z), "Tile Z" )
            ;

    po::variables_map vm ;
    po::store( po::parse_command_line(argc, argv, options), vm ) ;
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }



    // Convert the file
    const ctb::TileCoordinate coord(z, x, y);
    QuantizedMeshTile qmt(coord);
    if ( !qmt.convertFromHeightMapTile(inputFile) ) {
        cerr << "[ERROR] Cannot convert the input file" << endl ;
        return -1 ;
    }

    return 0 ;
}