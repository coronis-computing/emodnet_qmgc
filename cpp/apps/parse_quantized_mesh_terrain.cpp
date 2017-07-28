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
    std::string inputFile ;
    po::options_description options("Reads a quantized mesh terrain file") ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
    ;

    po::variables_map vm ;
    po::store( po::parse_command_line(argc, argv, options), vm ) ;
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Read the quantized mesh tile
    QuantizedMeshTile qmt ;
    if ( !qmt.readFile(inputFile) ) {
        cerr << "[ERROR] Cannot read the input file" << endl ;
        return -1 ;
    }


    return 1 ;
}