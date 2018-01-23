/**
 * @file
 * @brief Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format.
 * @author Ricard Campos (rcampos@eia.udg.edu)
 */

// Boost
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
// Std
#include <iostream>
#include <string>
#include <chrono>
#include <tin_creation/tin_creation_simplification_point_set_grid.h>
// Project-specific
#include "quantized_mesh_tiles_pyramid_builder_parallel.h"
#include "zoom_tiles_scheduler.h"
#include "ellipsoid.h"
#include "tin_creation/tin_creator.h"
#include "tin_creation/tin_creation_delaunay_strategy.h"
#include "tin_creation/tin_creation_simplification_lindstrom_turk_strategy.h"
#include "tin_creation/tin_creation_remeshing_strategy.h"
#include "tin_creation/tin_creation_greedy_insertion_strategy.h"
#include "tin_creation/tin_creation_simplification_point_set_hierarchy.h"
#include "tin_creation/tin_creation_simplification_point_set_wlop.h"
#include "tin_creation/tin_creation_simplification_point_set_grid.h"
#include "tin_creation/tin_creation_simplification_point_set_random.h"

using namespace std ;
namespace po = boost::program_options ;



int main ( int argc, char **argv)
{
    // Command line parser
    std::string inputFile, outDir, tinCreationStrategy, schedulerType, debugDir;
    int startZoom, endZoom;
    double simpWeightVolume, simpWeightBoundary, simpWeightShape, remeshingFacetDistance, remeshingFacetAngle, remeshingFacetSize, remeshingEdgeSize, psHierMaxSurfaceVariance, psBorderSimpMaxDist, greedyErrorTol, psWlopRetainPercentage, psWlopRadius, psGridCellSize, psRandomRemovePercentage;
    float clippingHighValue, clippingLowValue;
    int simpStopEdgesCount, heighMapSamplingSteps;
    unsigned int psHierMaxClusterSize, psWlopIterNumber;
    int numThreads = 0 ;
    bool bathymetryFlag = false ;

    po::options_description options("Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format");
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse" )
            ( "output-dir,o", po::value<std::string>(&outDir)->default_value("terrain_tiles_qm"), "The output directory for the tiles" )
            ( "start-zoom,s", po::value<int>(&startZoom)->default_value(-1), "The zoom level to start at. This should be greater than the end zoom level (i.e., the TMS pyramid is constructed from bottom to top). If smaller than zero, defaults to the maximum zoom possible according to DEM resolution." )
            ( "end-zoom,e", po::value<int>(&endZoom)->default_value(0), "The zoom level to end at. This should be less than the start zoom level (i.e., the TMS pyramid is constructed from bottom to top)." )
            ( "bathymetry,b", po::bool_switch(&bathymetryFlag), "Switch to consider the input DEM as containing depths instead of elevations" )
            ( "samples-per-tile", po::value<int>(&heighMapSamplingSteps)->default_value(256), "Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that a smaller sampling provides a coarser base mesh that will be easier to simplify." )
            ( "clip-high", po::value<float>(&clippingHighValue)->default_value(std::numeric_limits<float>::infinity()), "Clip values in the DEM above this threshold." )
            ( "clip-low", po::value<float>(&clippingLowValue)->default_value(-std::numeric_limits<float>::infinity()), "Clip values in the DEM below this threshold." )
            ( "num-threads", po::value<int>(&numThreads)->default_value(1), "Number of threads used (0=max_threads)" )
            ( "scheduler", po::value<string>(&schedulerType)->default_value("rowwise"), "Scheduler type. Defines the preferred tile processing order within a zoom. Note that on multithreaded executions this order may not be preserved. OPTIONS: rowwise, columnwise, chessboard, 4connected (see documentation for the meaning of each)" )

            ( "tc-strategy", po::value<string>(&tinCreationStrategy)->default_value("greedy"), "TIN creation strategy. OPTIONS: greedy, lt, delaunay, ps-hierarchy, ps-wlop, ps-grid, ps-random, remeshing, (see documentation for the meaning of each)" )

            ( "tc-greedy-error-tol", po::value<double>(&greedyErrorTol)->default_value(0.1), "Error tolerance for a tile to fulfill in the greedy insertion approach")

            ( "tc-lt-stop-edges-count", po::value<int>(&simpStopEdgesCount)->default_value(500), "Simplification stops when the number of edges is below this value." )
            ( "tc-lt-weight-volume", po::value<double>(&simpWeightVolume)->default_value(0.5), "Simplification volume weight (Lindstrom-Turk cost function, see original reference)." )
            ( "tc-lt-weight-boundary", po::value<double>(&simpWeightBoundary)->default_value(0.5), "Simplification boundary weight (Lindstrom-Turk cost function, see original reference)." )
            ( "tc-lt-weight-shape", po::value<double>(&simpWeightShape)->default_value(1e-10), "Simplification shape weight (Lindstrom-Turk cost function, see original reference)." )

            ( "tc-remeshing-facet-distance", po::value<double>(&remeshingFacetDistance)->default_value(0.2), "Remeshing facet distance." )
            ( "tc-remeshing-facet-angle", po::value<double>(&remeshingFacetAngle)->default_value(25), "Remeshing facet angle." )
            ( "tc-remeshing-facet-size", po::value<double>(&remeshingFacetSize)->default_value(0.2), "Remeshing facet size." )
            ( "tc-remeshing-edge-size", po::value<double>(&remeshingEdgeSize)->default_value(0.2), "Remeshing edge size." )

            ( "tc-ps-border-max-error", po::value<double>(&psBorderSimpMaxDist)->default_value(0.01), "Polyline simplification error at borders" )

            ( "tc-ps-hierarchy-cluster-size", po::value<unsigned int>(&psHierMaxClusterSize)->default_value(100), "Hierarchy point set simplification maximum cluster size" )
            ( "tc-ps-hierarchy-max-surface-variance", po::value<double>(&psHierMaxSurfaceVariance)->default_value(0.01), "Hierarchy point set simplification maximum surface variation" )

            ( "tc-ps-wlop-retain-percent", po::value<double>(&psWlopRetainPercentage)->default_value(5), "Percentage of points to retain, [0..100]" )
            ( "tc-ps-wlop-radius", po::value<double>(&psWlopRadius)->default_value(0.2), "PS WLOP simplification: radius" )
            ( "tc-ps-wlop-iter-number", po::value<unsigned int>(&psWlopIterNumber)->default_value(35), "PS WLOP simplification: number of iterations" )

            ( "tc-ps-grid-cell-size", po::value<double>(&psGridCellSize)->default_value(0.1), "PS Grid simplification: Cell size")

            ( "tc-ps-random-percent", po::value<double>(&psRandomRemovePercentage)->default_value(0.8), "PS Random simplification: percentage to remove")

            ( "debug-dir", po::value<string>(&debugDir)->default_value(""), "Debug directory where simplified meshes will be stored in OFF format for easing visualization")
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

    // --- Create as many tilers as required threads ---
    std::vector<QuantizedMeshTiler> tilers ;
    std::vector<GDALDataset *> gdalDatasets ;
    for ( int i = 0; i < numThreads; i++ ) {
        // Open the input dataset
        //GDALDataset *gdalDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
        gdalDatasets.push_back( (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly) );
        if (gdalDatasets[i] == NULL) {
            cerr << "Error: could not open GDAL dataset" << endl;
            return 1;
        }

        // Define the grid we are going to use
        int tileSize = 256;
        ctb::Grid grid = ctb::GlobalGeodetic(tileSize);

        // And the ellipsoid of reference
        Ellipsoid e;
        e = WGS84Ellipsoid();

        // The tiler options (default for the moment, let the user change this in the future?)
        ctb::TilerOptions gdalTilerOptions = ctb::TilerOptions();

        // The quantized mesh tiler options
        QuantizedMeshTiler::QMTOptions qmtOptions;
        qmtOptions.IsBathymetry = bathymetryFlag;
        qmtOptions.RefEllipsoid = e;
        qmtOptions.HeighMapSamplingSteps = heighMapSamplingSteps;
        qmtOptions.ClippingHighValue = clippingHighValue;
        qmtOptions.ClippingLowValue = clippingLowValue;

        // Setup the TIN creator
        TINCreator tinCreator;
        std::transform(tinCreationStrategy.begin(), tinCreationStrategy.end(), tinCreationStrategy.begin(), ::tolower);
        if (tinCreationStrategy.compare("lt") == 0) {
            std::shared_ptr<TINCreationSimplificationLindstromTurkStrategy> tcLT
                    = std::make_shared<TINCreationSimplificationLindstromTurkStrategy>(simpStopEdgesCount,
                                                                                       simpWeightVolume,
                                                                                       simpWeightBoundary,
                                                                                       simpWeightShape);
            tinCreator.setCreator(tcLT);
        }
        else if (tinCreationStrategy.compare("greedy") == 0) {
            std::shared_ptr<TinCreationGreedyInsertionStrategy> tcGreedy
                    = std::make_shared<TinCreationGreedyInsertionStrategy>(greedyErrorTol);
            tinCreator.setCreator(tcGreedy);
        }
        else if (tinCreationStrategy.compare("remeshing") == 0) {
            std::shared_ptr<TINCreationRemeshingStrategy> tcRemesh
                    = std::make_shared<TINCreationRemeshingStrategy>(remeshingFacetDistance,
                                                                   remeshingFacetAngle,
                                                                   remeshingFacetSize,
                                                                   remeshingEdgeSize);
            tinCreator.setCreator(tcRemesh);
        }
        else if (tinCreationStrategy.compare("ps-hierarchy") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetHierarchy> tcHier
                    = std::make_shared<TinCreationSimplificationPointSetHierarchy>(psBorderSimpMaxDist,
                                                                                 psHierMaxClusterSize,
                                                                                 psHierMaxSurfaceVariance);
            tinCreator.setCreator(tcHier);
        }
        else if (tinCreationStrategy.compare("ps-wlop") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetWLOP> tcWlop
                    = std::make_shared<TinCreationSimplificationPointSetWLOP>(psBorderSimpMaxDist,
                                                                              psWlopRetainPercentage,
                                                                              psWlopRadius,
                                                                              psWlopIterNumber);
            tinCreator.setCreator(tcWlop);
        }
        else if (tinCreationStrategy.compare("ps-grid") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetGrid> tcGrid
                    = std::make_shared<TinCreationSimplificationPointSetGrid>(psBorderSimpMaxDist,
                                                                              psGridCellSize);
            tinCreator.setCreator(tcGrid);
        }
        else if (tinCreationStrategy.compare("ps-random") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetRandom> tcRand
                    = std::make_shared<TinCreationSimplificationPointSetRandom>(psBorderSimpMaxDist,
                                                                                psRandomRemovePercentage);
            tinCreator.setCreator(tcRand);
        }
        else if (tinCreationStrategy.compare("delaunay") == 0) {
            std::shared_ptr<TINCreationDelaunayStrategy> tcDel =
                    std::make_shared<TINCreationDelaunayStrategy>();
            tinCreator.setCreator(tcDel);
        }
        else {
            std::cerr << "[ERROR] Unknown TIN creation strategy \"" << tinCreationStrategy << "\"" << std::endl;
            return 1;
        }

        // Create the tiler object
        QuantizedMeshTiler tiler(gdalDatasets[i], grid, gdalTilerOptions, qmtOptions, tinCreator);

        // Add the tiler
        tilers.push_back(tiler);
    }

    // Define the tiles' processing scheduler
    ZoomTilesScheduler scheduler ;
    std::transform(schedulerType.begin(), schedulerType.end(), schedulerType.begin(), ::tolower ) ;
    if (schedulerType.compare("rowwise") == 0) {
        std::shared_ptr<ZoomTilesSchedulerRowwiseStrategy> rowwiseScheduler
                = std::make_shared<ZoomTilesSchedulerRowwiseStrategy>() ;
        scheduler.setScheduler(rowwiseScheduler);
    }
    else if (schedulerType.compare("columnwise") == 0) {
        std::shared_ptr<ZoomTilesSchedulerColumnwiseStrategy> colwiseScheduler
                = std::make_shared<ZoomTilesSchedulerColumnwiseStrategy>() ;
        scheduler.setScheduler(colwiseScheduler) ;
    }
    else if (schedulerType.compare("4connected") == 0) {
        std::shared_ptr<ZoomTilesSchedulerFourConnectedStrategy> fourConnectedStrategy = std::make_shared<ZoomTilesSchedulerFourConnectedStrategy>() ;
        // std::shared_ptr<ZoomTilesSchedulerRecursiveFourConnectedStrategy> fourConnectedStrategy = std::make_shared<ZoomTilesSchedulerRecursiveFourConnectedStrategy>() ;
        scheduler.setScheduler(fourConnectedStrategy) ;
    }
    else if (schedulerType.compare("chessboard") == 0) {
        std::shared_ptr<ZoomTilesSchedulerChessboardStrategy> chessboardScheduler = std::make_shared<ZoomTilesSchedulerChessboardStrategy>() ;
        scheduler.setScheduler(chessboardScheduler);
    }
    else {
        std::cerr << "[ERROR] Unknown scheduler type \"" << schedulerType << "\"" << std::endl;
        return 1;
    }

    // Create the output directory, if needed
    fs::path outDirPath( outDir ) ;
    if ( !fs::exists( outDirPath ) && !fs::create_directory( outDirPath ) ) {
        cerr << "[ERROR] Cannot create the output folder" << outDirPath << endl ;
        return 1 ;
    }

    // Create the tiles
    QuantizedMeshTilesPyramidBuilderParallel qmtpb( tilers, scheduler ) ;
    auto start = std::chrono::high_resolution_clock::now();
    qmtpb.createTmsPyramid( startZoom, endZoom, outDir, debugDir ) ;
    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Requested tiles created in " << elapsed.count() << " seconds" << std::endl
              << "Remember to create a layer.json file in the root folder! (see ""create_layer_json.py script"")"
              << std::endl ;

    // Delete gdalDatasets pointers;
    for (std::vector<GDALDataset *>::iterator it = gdalDatasets.begin() ; it != gdalDatasets.end(); ++it)
    {
        delete (*it);
    }

    return 0 ;
}