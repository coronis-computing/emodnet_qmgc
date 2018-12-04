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
 * @brief Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format.
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// Boost
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
// Std
#include <iostream>
#include <string>
#include <chrono>
#include <future>
// GDAl
#include "cpl_conv.h"
#include <ogrsf_frmts.h>
// Project-specific
#include "quantized_mesh_tiles_pyramid_builder.h"
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

using namespace std;
using namespace TinCreation;
namespace po = boost::program_options;

namespace std
{
    /**
    * @brief Adding ostream operator << to vectors. Required by boost::program_options to be able to define default values for vectors
    */
    std::ostream& operator<<(std::ostream &os, const std::vector<double> &vec)
    {
        for (auto item : vec)
        {
            os << item << " ";
        }
        return os;
    }

    std::ostream& operator<<(std::ostream &os, const std::vector<int> &vec)
    {
        for (auto item : vec)
        {
            os << item << " ";
        }
        return os;
    }

    std::ostream& operator<<(std::ostream &os, const std::vector<unsigned int> &vec)
    {
        for (auto item : vec)
        {
            os << item << " ";
        }
        return os;
    }
}

int main ( int argc, char **argv)
{
    // Command line parser
    std::string inputFile, outDir, tinCreationStrategy, schedulerType, debugDir, configFile, greedyErrorType;
    int startZoom, endZoom;
    double simpWeightVolume, simpWeightBoundary, simpWeightShape, remeshingFacetAngle;
    float clippingHighValue, clippingLowValue, belowSeaLevelScaleFactor, aboveSeaLevelScaleFactor;
    int heighMapSamplingSteps, greedyInitGridSize;
    unsigned int psWlopIterNumber, psMinFeaturePolylineSize;
    int numThreads = 0 ;
    bool bathymetryFlag ;
    // Parameters per zoom level
    std::vector<int> simpStopEdgesCount;
    std::vector<unsigned int> psHierMaxClusterSize;
    std::vector<double> greedyErrorTol, remeshingFacetDistance, remeshingFacetSize, remeshingEdgeSize, psBorderSimpMaxDist, psBorderSimpMaxLength, psHierMaxSurfaceVariance, psWlopRetainPercentage, psWlopRadius, psGridCellSize, psRandomRemovePercentage;

    po::options_description options("qm_tiler options");
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input terrain file to parse (can be specified like this or as a positional parameter)" )
            ( "output-dir,o", po::value<std::string>(&outDir)->default_value("terrain_tiles_qm"), "The output directory for the tiles" )
            ( "start-zoom,s", po::value<int>(&startZoom)->default_value(-1), "The zoom level to start at. This should be greater than the end zoom level (i.e., the TMS pyramid is constructed from bottom to top). If smaller than zero, defaults to the maximum zoom possible according to DEM resolution." )
            ( "end-zoom,e", po::value<int>(&endZoom)->default_value(0), "The zoom level to end at. This should be less than the start zoom level (i.e., the TMS pyramid is constructed from bottom to top)." )
            ( "bathymetry,b", po::value<bool>(&bathymetryFlag)->default_value(false), "Switch to consider the input DEM as containing depths instead of elevations" )
            ( "samples-per-tile", po::value<int>(&heighMapSamplingSteps)->default_value(256), "Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that a smaller sampling provides a coarser base mesh that will be easier to simplify." )
            ( "clip-high", po::value<float>(&clippingHighValue)->default_value(std::numeric_limits<float>::infinity()), "Clip values in the DEM above this threshold." )
            ( "clip-low", po::value<float>(&clippingLowValue)->default_value(-std::numeric_limits<float>::infinity()), "Clip values in the DEM below this threshold." )
            ( "above-sea-level-scale-factor", po::value<float>(&aboveSeaLevelScaleFactor)->default_value(-1), "Scale factor to apply to the readings above sea level (ignored if < 0)" )
            ( "below-sea-level-scale-factor", po::value<float>(&belowSeaLevelScaleFactor)->default_value(-1), "Scale factor to apply to the readings below sea level (ignored if < 0)" )
            ( "num-threads", po::value<int>(&numThreads)->default_value(1), "Number of threads used (0=max_threads)" )
            ( "scheduler", po::value<string>(&schedulerType)->default_value("rowwise"), "Scheduler type. Defines the preferred tile processing order within a zoom. Note that on multithreaded executions this order may not be preserved. OPTIONS: rowwise, columnwise, chessboard, 4connected (see documentation for the meaning of each)" )
            ( "tc-strategy", po::value<string>(&tinCreationStrategy)->default_value("greedy"), "TIN creation strategy. OPTIONS: greedy, lt, delaunay, ps-hierarchy, ps-wlop, ps-grid, ps-random (see documentation for further information)" )
            ( "tc-greedy-error-tol", po::value<vector<double> >(&greedyErrorTol)->multitoken()->default_value(vector<double>{150000}), "Error tolerance for a tile to fulfill in the greedy insertion approach (*).")
            ( "tc-greedy-init-grid-size", po::value<int>(&greedyInitGridSize)->default_value(-1), "An initial grid of this size will be used as base mesh to start the insertion process. Defaults to the 4 corners of the tile if < 0")
            ( "tc-greedy-error-type", po::value<string>(&greedyErrorType)->default_value("height"), "The error computation type. Available: height, 3d.")
            ( "tc-lt-stop-edges-count", po::value<vector<int> >(&simpStopEdgesCount)->multitoken()->default_value(vector<int>{500}), "Simplification stops when the number of edges is below this value (*)." )
            ( "tc-lt-weight-volume", po::value<double>(&simpWeightVolume)->default_value(0.5), "Simplification volume weight (Lindstrom-Turk cost function, see original reference)." )
            ( "tc-lt-weight-boundary", po::value<double>(&simpWeightBoundary)->default_value(0.5), "Simplification boundary weight (Lindstrom-Turk cost function, see original reference)." )
            ( "tc-lt-weight-shape", po::value<double>(&simpWeightShape)->default_value(1e-10), "Simplification shape weight (Lindstrom-Turk cost function, see original reference)." )
// For the moment, we eliminate the remeshing mode for tiles generation
//            ( "tc-remeshing-facet-distance", po::value<vector<double> >(&remeshingFacetDistance)->multitoken()->default_value(vector<double>{150000}), "Remeshing facet distance threshold." )
//            ( "tc-remeshing-facet-angle", po::value<double>(&remeshingFacetAngle)->default_value(25), "Remeshing facet angle threshold." )
//            ( "tc-remeshing-facet-size", po::value<vector<double> >(&remeshingFacetSize)->multitoken()->default_value(vector<double>{150000}), "Remeshing facet size threshold." )
//            ( "tc-remeshing-edge-size", po::value<vector<double> >(&remeshingEdgeSize)->multitoken()->default_value(vector<double>{150000}), "Remeshing edge size threshold." )
            ( "tc-ps-border-max-error", po::value<vector<double> >(&psBorderSimpMaxDist)->multitoken()->default_value(vector<double>{10000}), "Polyline simplification error at borders (*)." )
            ( "tc-ps-border-max-length-xy-percent", po::value<vector<double> >(&psBorderSimpMaxLength)->multitoken()->default_value(std::vector<double>{20}), "Polyline simplification, maximum length of border edges when projected to the XY plane. Expressed as a percentage [0..100] (*)." )
            ( "tc-ps-features-min-size", po::value<unsigned int>(&psMinFeaturePolylineSize)->default_value(5), "Minimum number of points in a feature polyline to be considered." )
            ( "tc-ps-hierarchy-cluster-size", po::value<vector<unsigned int> >(&psHierMaxClusterSize)->default_value(vector<unsigned int>{1000}), "Hierarchy point set simplification maximum cluster size (*)." )
            ( "tc-ps-hierarchy-max-surface-variance", po::value<vector<double> >(&psHierMaxSurfaceVariance)->default_value(vector<double>{1000}), "Hierarchy point set simplification maximum surface variation (*)." )
            ( "tc-ps-wlop-retain-percent", po::value<vector<double> >(&psWlopRetainPercentage)->multitoken()->default_value(vector<double>{1}), "Percentage of points to retain, [0..100] (*)." )
            ( "tc-ps-wlop-radius", po::value<vector<double> >(&psWlopRadius)->multitoken()->default_value(vector<double>{10000}), "PS WLOP simplification: radius." )
            ( "tc-ps-wlop-iter-number", po::value<unsigned int>(&psWlopIterNumber)->default_value(35), "PS WLOP simplification: number of iterations." )
            ( "tc-ps-grid-cell-size", po::value<vector<double> >(&psGridCellSize)->multitoken()->default_value(vector<double>{10000}), "PS Grid simplification: Cell size (*).")
            ( "tc-ps-random-percent", po::value<vector<double> >(&psRandomRemovePercentage)->multitoken()->default_value(vector<double>{90}), "PS Random simplification: percentage to remove.")
            ( "debug-dir", po::value<string>(&debugDir)->default_value(""), "Debug directory where simplified meshes will be stored in OFF format for easing visualization.")
            ( "config,c", po::value<string>(&configFile)->default_value(""), "Configuration file with a set of the options above specified in the form <option>=<value>. Note that the options in the config file have preference over the ones specified on the command line.")
    ;
    po::positional_options_description positionalOptions;
    positionalOptions.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(options).positional(positionalOptions).run(), vm);

    // Read the configuration file (if exists)
    if(vm.count("config") > 0 && !vm["config"].as<std::string>().empty()) {
        configFile = vm["config"].as<std::string>() ;
        ifstream ifs(configFile);
        if (ifs.good())
            po::store(po::parse_config_file(ifs, options), vm);
        else {
            cerr << "[Error] Could not open config file " << configFile << endl;
            return 1;
        }
    }

    po::notify(vm);

    if (vm.count("help")) {
        cout << "Creates the tiles of a GDAL raster terrain in Cesium's Quantized Mesh format\n\n"
                "(*) The parameters marked with this sign can be specified multiple times to reflect the desired value per zoom level.\n"
                "When the required zoom is larger than the number of specified parameters, the last one is used.\n"
                "On the other hand, when a single parameter is specified, it is assumed to represent the value for the root level of the pyramid (i.e., zoom=0),\n"
                "and the values for the deeper levels will be computed as value/2^zoom, for those parameters whose scale needs to be lowered at deeper levels,\n"
                "or value*2^zoom, for those whose scale needs to grow with depth.\n\n"
             << options << "\n";
        return 1;
    }

    bool preserveBorders = tinCreationStrategy.compare("delaunay") != 0;

    // Setup all GDAL-supported raster drivers
    GDALAllRegister();
    CPLSetConfigOption("VRT_SHARED_SOURCE", "0"); // Needed when accessing a single VRT from multiple threads: http://gdal.org/gdal_vrttut.html#gdal_vrttut_mt

    // --- Create as many tilers as required threads ---
    std::vector<QuantizedMeshTiler> tilers ;
    std::vector<GDALDataset *> gdalDatasets ;
    if (numThreads == 0)
        numThreads = std::thread::hardware_concurrency();
    for ( int i = 0; i < numThreads; i++ ) {
        // Open the input dataset
        //GDALDataset *gdalDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
        gdalDatasets.push_back( (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly) );
        if (gdalDatasets[i] == NULL) {
            cerr << "[Error] Could not open GDAL dataset" << endl;
            return EXIT_FAILURE;
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
        qmtOptions.AboveSeaLevelScaleFactor = aboveSeaLevelScaleFactor;
        qmtOptions.BelowSeaLevelScaleFactor = belowSeaLevelScaleFactor;

        // Setup the TIN creator
        TinCreator tinCreator;
        std::transform(tinCreationStrategy.begin(), tinCreationStrategy.end(), tinCreationStrategy.begin(), ::tolower);
        if (tinCreationStrategy.compare("lt") == 0) {
            std::shared_ptr<TinCreationSimplificationLindstromTurkStrategy> tcLT
                    = std::make_shared<TinCreationSimplificationLindstromTurkStrategy>(simpStopEdgesCount,
                                                                                       simpWeightVolume,
                                                                                       simpWeightBoundary,
                                                                                       simpWeightShape);
            tinCreator.setCreator(tcLT);
        }
        else if (tinCreationStrategy.compare("greedy") == 0) {
            std::transform(greedyErrorType.begin(), greedyErrorType.end(), greedyErrorType.begin(), ::tolower);
            int et;
            if (greedyErrorType.compare("height") == 0)
                et = TinCreationGreedyInsertionStrategy::ErrorHeight;
            else if (greedyErrorType.compare("3d") == 0)
                et = TinCreationGreedyInsertionStrategy::Error3D;
            else {
                std::cerr << "[ERROR] Unknown error type \"" << tinCreationStrategy << "\" for the Greedy TIN creation strategy" << std::endl;
                return 1;
            }

            std::shared_ptr<TinCreationGreedyInsertionStrategy> tcGreedy
                    = std::make_shared<TinCreationGreedyInsertionStrategy>(greedyErrorTol, greedyInitGridSize, et);
            tinCreator.setCreator(tcGreedy);
        }
//        else if (tinCreationStrategy.compare("remeshing") == 0) {
//            std::shared_ptr<TinCreationRemeshingStrategy> tcRemesh
//                    = std::make_shared<TinCreationRemeshingStrategy>(remeshingFacetDistance,
//                                                                     remeshingFacetAngle,
//                                                                     remeshingFacetSize,
//                                                                     remeshingEdgeSize);
//            tinCreator.setCreator(tcRemesh);
//        }
        else if (tinCreationStrategy.compare("ps-hierarchy") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetHierarchy> tcHier
                    = std::make_shared<TinCreationSimplificationPointSetHierarchy>(psBorderSimpMaxDist,
                                                                                   psBorderSimpMaxLength,
                                                                                   psMinFeaturePolylineSize,
                                                                                   psHierMaxClusterSize,
                                                                                   psHierMaxSurfaceVariance);
            tinCreator.setCreator(tcHier);
        }
        else if (tinCreationStrategy.compare("ps-wlop") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetWLOP> tcWlop
                    = std::make_shared<TinCreationSimplificationPointSetWLOP>(psBorderSimpMaxDist,
                                                                              psBorderSimpMaxLength,
                                                                              psMinFeaturePolylineSize,
                                                                              psWlopRetainPercentage,
                                                                              psWlopRadius,
                                                                              psWlopIterNumber);
            tinCreator.setCreator(tcWlop);
        }
        else if (tinCreationStrategy.compare("ps-grid") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetGrid> tcGrid
                    = std::make_shared<TinCreationSimplificationPointSetGrid>(psBorderSimpMaxDist,
                                                                              psBorderSimpMaxLength,
                                                                              psMinFeaturePolylineSize,
                                                                              psGridCellSize);
            tinCreator.setCreator(tcGrid);
        }
        else if (tinCreationStrategy.compare("ps-random") == 0) {
            std::shared_ptr<TinCreationSimplificationPointSetRandom> tcRand
                    = std::make_shared<TinCreationSimplificationPointSetRandom>(psBorderSimpMaxDist,
                                                                                psBorderSimpMaxLength,
                                                                                psMinFeaturePolylineSize,
                                                                                psRandomRemovePercentage);
            tinCreator.setCreator(tcRand);
        }
        else if (tinCreationStrategy.compare("delaunay") == 0) {
            std::shared_ptr<TinCreationDelaunayStrategy> tcDel =
                    std::make_shared<TinCreationDelaunayStrategy>();
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
        std::shared_ptr<ZoomTilesSchedulerFourConnectedStrategy> fourConnectedStrategy
                = std::make_shared<ZoomTilesSchedulerFourConnectedStrategy>();
//        std::shared_ptr<ZoomTilesSchedulerRecursiveFourConnectedStrategy> fourConnectedStrategy = std::make_shared<ZoomTilesSchedulerRecursiveFourConnectedStrategy>() ;
        scheduler.setScheduler(fourConnectedStrategy) ;
    }
    else if (schedulerType.compare("chessboard") == 0) {
        std::shared_ptr<ZoomTilesSchedulerChessboardStrategy> chessboardScheduler
                = std::make_shared<ZoomTilesSchedulerChessboardStrategy>();
        scheduler.setScheduler(chessboardScheduler);
    }
    else {
        std::cerr << "[ERROR] Unknown scheduler type \"" << schedulerType << "\"" << std::endl;
        return EXIT_FAILURE;
    }

    // Create the output directory, if needed
    fs::path outDirPath(outDir) ;
    if (!fs::exists(outDirPath) && !fs::create_directory(outDirPath)) {
        cerr << "[ERROR] Cannot create the output folder" << outDirPath << endl ;
        return EXIT_FAILURE;
    }

    // Create the tiles
    QuantizedMeshTilesPyramidBuilder qmtpb(tilers, scheduler);
    auto start = std::chrono::high_resolution_clock::now();
    if (preserveBorders)
        qmtpb.createTmsPyramid(startZoom, endZoom, outDir, debugDir);
    else
        qmtpb.createTmsPyramidUnconstrainedBorders(startZoom, endZoom, outDir, debugDir);
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

    return EXIT_SUCCESS;
}