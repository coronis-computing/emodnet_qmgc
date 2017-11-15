//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

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