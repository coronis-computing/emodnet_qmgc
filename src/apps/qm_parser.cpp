/**
 * @file
 * @brief Reads a quantized mesh terrain file
 *
 * @author Ricard Campos (rcampos@eia.udg.edu)
 *
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
    bool headerOnly ;
    po::options_description options("Reads a quantized mesh terrain file") ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
            ( "output,o", po::value<std::string>(&outputFile), "Output OFF file (Optional). The tile can be converted to this format to ease visualization in common 3D viewers" )
            ( "tileX,x", po::value<int>(&x), "Tile X" )
            ( "tileY,y", po::value<int>(&y), "Tile Y" )
            ( "tileZ,z", po::value<int>(&z), "Tile Zoom" )
            ( "header", po::value<bool>(&headerOnly)->default_value(false), "Flag to show only the header of the tile on screen")
    ;

    po::variables_map vm ;
    po::store( po::parse_command_line(argc, argv, options), vm ) ;
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Read the quantized mesh tile
    const ctb::TileCoordinate coord(z, x, y);
    QuantizedMeshTile qmt(coord);
    if ( !qmt.readFile(inputFile) ) {
        cerr << "[ERROR] Cannot read the input file" << endl ;
        return -1 ;
    }

    // Print information on screen
    if (headerOnly)
        qmt.printHeader() ;
    else
        qmt.print() ;

    // Export to OFF
    if (!outputFile.empty())
        qmt.exportToOFF( outputFile, false ) ;

    return 1 ;
}