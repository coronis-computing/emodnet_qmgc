//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

// Boost
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
// Std
#include <iostream>
#include <string>
// Project-specific
#include "quantized_mesh_tile.h"
#include "quantized_mesh_tiler.h"
#include "ellipsoid.h"

using namespace std ;
namespace po = boost::program_options ;



int main ( int argc, char **argv)
{
    // Command line parser
    std::string inputFile, outDir ;
    int startZoom, endZoom ;
    po::options_description options("Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format") ;
    bool bathymetryFlag = false;
    double simpCountRatioStop ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
            ( "output-dir,o", po::value<std::string>(&outDir)->default_value("terrain_tiles_qm"), "The output directory for the tiles" )
            ( "start-zoom,s", po::value<int>(&startZoom)->default_value(-1), "The zoom level to start at. This should be greater than the end zoom level")
            ( "end-zoom,e", po::value<int>(&endZoom)->default_value(-1), "The zoom level to end at. This should be less than the start zoom level and >= 0. If smaller than zero, defaults to max_zoom")
            ( "bathymetry,b", po::bool_switch(&bathymetryFlag), "Switch to consider the input DEM as containing depths instead of elevations")
            ( "stop-ratio,r", po::value<double>(&simpCountRatioStop)->default_value(0.05), "Simplification stops when the relation between the initial and current number of edges drops below this ratio")
    ;
    po::positional_options_description positionalOptions;
    positionalOptions.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(options).positional(positionalOptions).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Setup all GDAL-supported raster drivers
    GDALAllRegister();

    // Open the input dataset
    GDALDataset  *poDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
    if (poDataset == NULL) {
        cerr << "Error: could not open GDAL dataset" << endl;
        return 1;
    }

    // Define the grid we are going to use
    int tileSize = 256 ; // TODO: Check if this is ok...
    ctb::Grid grid = ctb::GlobalGeodetic(tileSize);

    // And the ellipsoid of reference
    Ellipsoid e ;
    e = WGS84Ellipsoid() ;

    // The tiler options (default for the moment, let the user change this in the future?)
    ctb::TilerOptions to = ctb::TilerOptions() ;

    // Create the tiler object
    QuantizedMeshTiler tiler(poDataset, grid, to, bathymetryFlag, e, simpCountRatioStop);

    // Create the output directory, if needed
    fs::path outDirPath( outDir ) ;
    if ( !fs::exists( outDirPath ) && !fs::create_directory( outDirPath ) ) {
        cerr << "[ERROR] Cannot create the output folder" << outDirPath << endl ;
        return -1 ;
    }

    // Set the desired zoom levels to process
    startZoom = (startZoom < 0) ? tiler.maxZoomLevel() : startZoom ;
    endZoom = (endZoom < 0) ? 0 : endZoom;

    // Create the tiles
    tiler.createTilePyramid(startZoom, endZoom, outDir) ;

    std::cout << "TMS pyramid created." << std::endl << "Remember to create a layer.json file in the root folder! (see ""create_layer_json.py script"")" << std::endl ;

    return 0 ;
}