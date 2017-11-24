//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

// Boost
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
// Std
#include <iostream>
#include <string>
#include <zoom_tiles_processing_scheduler.h>
// Project-specific
//#include "quantized_mesh_tile.h"
//#include "quantized_mesh_tiler.h"
#include "quantized_mesh_tiles_pyramid_builder.h"
#include "zoom_tiles_processing_scheduler.h"
#include "ellipsoid.h"

using namespace std ;
namespace po = boost::program_options ;



int main ( int argc, char **argv)
{
    // Command line parser
    std::string inputFile, outDir ;
    int startZoom, endZoom ;
    po::options_description options("Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format") ;
    bool bathymetryFlag, noSimplify;
    double simpStopCost, simpWeightVolume, simpWeightBoundary, simpWeightShape ;
    float clippingHighValue, clippingLowValue ;
    int simpStopEdgesCount ;
    int numThreads = 0 ;
    int heighMapSamplingSteps ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
            ( "output-dir,o", po::value<std::string>(&outDir)->default_value("terrain_tiles_qm"), "The output directory for the tiles" )
            ( "start-zoom,s", po::value<int>(&startZoom)->default_value(-1), "The zoom level to start at. This should be greater than the end zoom level (i.e., the TMS pyramid is constructed from bottom to top). If smaller than zero, defaults to the maximum zoom possible according to DEM resolution.")
            ( "end-zoom,e", po::value<int>(&endZoom)->default_value(0), "The zoom level to end at. This should be less than the start zoom level (i.e., the TMS pyramid is constructed from bottom to top).")
            ( "bathymetry,b", po::bool_switch(&bathymetryFlag), "Switch to consider the input DEM as containing depths instead of elevations")
            ( "no-simp", po::bool_switch(&noSimplify), "Flag disabling simplification. The terrain will be represented with a regular grid extracted from the rasters (similar to the old heightmap format)")
            ( "samples-per-tile", po::value<int>(&heighMapSamplingSteps)->default_value(256), "Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that a smaller sampling provides a coarser base mesh that will be easier to simplify.")
            ( "simp-stop-edges-count", po::value<int>(&simpStopEdgesCount)->default_value(500), "Simplification stops when the number of edges is below this value.")
            ( "simp-weight-volume", po::value<double>(&simpWeightVolume)->default_value(0.5), "Simplification volume weight (Lindstrom-Turk cost function, see original reference).")
            ( "simp-weight-boundary", po::value<double>(&simpWeightBoundary)->default_value(0.5), "Simplification boundary weight (Lindstrom-Turk cost function, see original reference).")
            ( "simp-weight-shape", po::value<double>(&simpWeightShape)->default_value(1e-10), "Simplification shape weight (Lindstrom-Turk cost function, see original reference).")
            ( "clip-high", po::value<float>(&clippingHighValue)->default_value(std::numeric_limits<float>::infinity()), "Clip values in the DEM above this threshold.")
            ( "clip-low", po::value<float>(&clippingLowValue)->default_value(-std::numeric_limits<float>::infinity()), "Clip values in the DEM below this threshold.")
#ifdef USE_OPENMP
            ("num-threads", po::value<int>(&numThreads)->default_value(1), "Number of threads used (0=max_threads)")
#endif
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
//    GDALDataset  *poDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
//    if (poDataset == NULL) {
//        cerr << "Error: could not open GDAL dataset" << endl;
//        return 1;
//    }

    // Define the grid we are going to use
//    int tileSize = 256 ; // TODO: Check if this is ok...
//    ctb::Grid grid = ctb::GlobalGeodetic(tileSize);

    // And the ellipsoid of reference
    Ellipsoid e ;
    e = WGS84Ellipsoid() ;

    // The tiler options (default for the moment, let the user change this in the future?)
    ctb::TilerOptions gdalTilerOptions = ctb::TilerOptions() ;

    // The quantized mesh tiler options
    QuantizedMeshTiler::QMTOptions qmtOptions ;
    qmtOptions.IsBathymetry = bathymetryFlag ;
    qmtOptions.RefEllipsoid = e ;
    qmtOptions.HeighMapSamplingSteps = heighMapSamplingSteps ;
    qmtOptions.Simplify = !noSimplify ;
    qmtOptions.SimpStopEdgesCount = simpStopEdgesCount ;
    qmtOptions.SimpWeightVolume = simpWeightVolume ;
    qmtOptions.SimpWeightBoundary = simpWeightBoundary ;
    qmtOptions.SimpWeightShape = simpWeightShape ;
    qmtOptions.ClippingHighValue = clippingHighValue ;
    qmtOptions.ClippingLowValue = clippingLowValue ;

    // The tiles' processing scheduler
    ZoomTilesProcessingSchedulerBase* scheduler ;
    scheduler = new ZoomTilesProcessingSchedulerRowwise() ;

    // Create the tiler object
//    QuantizedMeshTiler tiler(poDataset, grid, to, bathymetryFlag, e, simpCountRatioStop, numThreads, inputFile);
//    QuantizedMeshTiler tiler(poDataset, grid, to, bathymetryFlag, e, simpCountRatioStop);

    // Create the output directory, if needed
    fs::path outDirPath( outDir ) ;
    if ( !fs::exists( outDirPath ) && !fs::create_directory( outDirPath ) ) {
        cerr << "[ERROR] Cannot create the output folder" << outDirPath << endl ;
        return -1 ;
    }


//
//    // Create the tiles
//    tiler.createTilePyramid(startZoom, endZoom, outDir) ;
    QuantizedMeshTilesPyramidBuilder qmtpb( inputFile, gdalTilerOptions, qmtOptions, scheduler, numThreads ) ;
    qmtpb.createTmsPyramid( startZoom, endZoom, outDir ) ;

    std::cout << "TMS pyramid created." << std::endl << "Remember to create a layer.json file in the root folder! (see ""create_layer_json.py script"")" << std::endl ;

    return 0 ;
}