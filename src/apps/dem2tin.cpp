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
 * @brief Creates a Triangulated Irregular Network (TIN) out of a raster digital elevation model (DEM).
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// Std
#include <iostream>
#include <string>
#include <chrono>
// GDAL
#include "gdal_priv.h"
#include <ogrsf_frmts.h>
// Boost
#include <boost/program_options.hpp>
// CGAL
#include "tin_creation/tin_creation_cgal_types.h"
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/bounding_box.h>
// Tin creation
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



int main ( int argc, char **argv) {
    // Command line parser
    std::string inputFile, inputType, outputFile, tinCreationStrategy, schedulerType, debugDir, configFile, greedyErrorType, exportNewOrigin;
    double greedyErrorTol, simpWeightVolume, simpWeightBoundary, simpWeightShape,
           remeshingFacetDistance, remeshingFacetAngle, remeshingFacetSize, remeshingEdgeSize,
           psBorderSimpMaxDist, psBorderSimpMaxLength, psHierMaxSurfaceVariance, psWlopRetainPercentage, psWlopRadius, psGridCellSize,
           psRandomRemovePercentage ;
    float clippingHighValue, clippingLowValue;
    int simpStopEdgesCount, heighMapSamplingSteps, greedyInitGridSize;
    unsigned int psHierMaxClusterSize, psWlopIterNumber, psMinFeaturePolylineSize;
    bool bathymetryFlag, verbose, resetOrigin;

    po::options_description options("dem2tin options");
    options.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<std::string>(&inputFile), "Input terrain file (GDAL raster)")
            ("input-type", po::value<std::string>(&inputType)->default_value("gdal"), "Type of the input terrain file. Options: gdal, point-set, off")
            ("output,o", po::value<std::string>(&outputFile)->default_value("./out.off"), "Output OFF file with the TIN")
            ("bathymetry,b", po::value<bool>(&bathymetryFlag)->default_value(false), "Switch to consider the input DEM as containing depths instead of elevations (only for GDAL files)")
            ("clip-high", po::value<float>(&clippingHighValue)->default_value(std::numeric_limits<float>::infinity()), "Clip values in the DEM above this threshold (only for GDAL files)")
            ("clip-low", po::value<float>(&clippingLowValue)->default_value(-std::numeric_limits<float>::infinity()), "Clip values in the DEM below this threshold (only for GDAL files)")
            ("reset-origin", po::value<bool>(&resetOrigin)->default_value(true), "Resets the origin of the input samples to be minimum point of their bounding box. Just used when the input is not a GDAL raster")
            ("export-new-origin", po::value<std::string>(&exportNewOrigin)->default_value(""), "Path where to export the new origin. Only use when reset-origin parameter is set to true. The format is the minimum X, Y, Z of the input vertices. Adding this values to the resulting vertices should get them in the original coordinates.")
            ("tc-strategy", po::value<string>(&tinCreationStrategy)->default_value("greedy"), "TIN creation strategy. OPTIONS: greedy, lt, delaunay, ps-hierarchy, ps-wlop, ps-grid, ps-random, remeshing, (see documentation for the meaning of each)" )
            ("tc-greedy-error-tol", po::value<double>(&greedyErrorTol)->default_value(0.1), "Error tolerance for a tile to fulfill in the greedy insertion approach")
            ("tc-greedy-init-grid-size", po::value<int>(&greedyInitGridSize)->default_value(-1), "An initial grid of this size will be used as base mesh to start the insertion process. Defaults to the 4 corners of the terrain if < 0")
            ("tc-greedy-error-type", po::value<string>(&greedyErrorType)->default_value("height"), "The error computation type. Available: height, 3d.")
            ("tc-lt-stop-edges-count", po::value<int>(&simpStopEdgesCount)->default_value(500), "Simplification stops when the number of edges is below this value." )
            ("tc-lt-weight-volume", po::value<double>(&simpWeightVolume)->default_value(0.5), "Simplification volume weight (Lindstrom-Turk cost function, see original reference)." )
            ("tc-lt-weight-boundary", po::value<double>(&simpWeightBoundary)->default_value(0.5), "Simplification boundary weight (Lindstrom-Turk cost function, see original reference)." )
            ("tc-lt-weight-shape", po::value<double>(&simpWeightShape)->default_value(1e-10), "Simplification shape weight (Lindstrom-Turk cost function, see original reference)." )
// Remeshing is experimental, do not use yet...
//            ("tc-remeshing-facet-distance", po::value<double>(&remeshingFacetDistance)->default_value(10), "Remeshing facet distance." )
//            ("tc-remeshing-facet-angle", po::value<double>(&remeshingFacetAngle)->default_value(25), "Remeshing facet angle." )
//            ("tc-remeshing-facet-size", po::value<double>(&remeshingFacetSize)->default_value(10), "Remeshing facet size." )
//            ("tc-remeshing-edge-size", po::value<double>(&remeshingEdgeSize)->default_value(10), "Remeshing edge size." )
            ("tc-ps-border-max-error", po::value<double>(&psBorderSimpMaxDist)->default_value(0.01), "Polyline simplification error at borders" )
            ("tc-ps-border-max-length", po::value<double>(&psBorderSimpMaxLength)->default_value(0.1), "Polyline simplification, maximum length of border edges" )
            ("tc-ps-features-min-size", po::value<unsigned int>(&psMinFeaturePolylineSize)->default_value(5), "Minimum number of points in a feature polyline to be considered" )
            ("tc-ps-hierarchy-cluster-size", po::value<unsigned int>(&psHierMaxClusterSize)->default_value(100), "Hierarchy point set simplification maximum cluster size" )
            ("tc-ps-hierarchy-max-surface-variance", po::value<double>(&psHierMaxSurfaceVariance)->default_value(0.01), "Hierarchy point set simplification maximum surface variation" )
            ("tc-ps-wlop-retain-percent", po::value<double>(&psWlopRetainPercentage)->default_value(5), "Percentage of points to retain, [0..100]" )
            ("tc-ps-wlop-radius", po::value<double>(&psWlopRadius)->default_value(0.2), "PS WLOP simplification: radius" )
            ("tc-ps-wlop-iter-number", po::value<unsigned int>(&psWlopIterNumber)->default_value(35), "PS WLOP simplification: number of iterations" )
            ("tc-ps-grid-cell-size", po::value<double>(&psGridCellSize)->default_value(0.1), "PS Grid simplification: Cell size")
            ("tc-ps-random-percent", po::value<double>(&psRandomRemovePercentage)->default_value(80), "PS Random simplification: percentage to remove")
            ("verbose", po::value<bool>(&verbose)->default_value(true), "Activate/deactivate output on the screen")
            ("config,c", po::value<string>(&configFile)->default_value(""), "Configuration file with a set of the options above specified in the form <option>=<value>. Note that the options in the config file have preference over the ones specified on the command line.")
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
        cout << "Creates a Triangulated Irregular Network (TIN) from a GDAL raster or from another TIN\n\n"
             << options << endl;
        return EXIT_FAILURE;
    }

    bool usingAMethodRequiringECEF = tinCreationStrategy.compare("ps-hierarchy") == 0 ||
                                     tinCreationStrategy.compare("ps-wlop") == 0 ||
                                     tinCreationStrategy.compare("ps-grid") == 0 ||
                                     tinCreationStrategy.compare("ps-random") == 0;
                                     // tinCreationStrategy.compare("remeshing") == 0
    if (usingAMethodRequiringECEF && inputType.compare("gdal") == 0) {
        std::cout << "For the moment, using a point set processing method on a gdal raster is not possible.\n"
                     "If you want to use these methods, please use an input point set or TIN in METRIC units\n"
                     "Aborting..." << std::endl;
        return EXIT_FAILURE;
    }

    // Read the input samples
    std::vector<Point_3> samples ;
    double minX, minY, minHeight, maxX, maxY, maxHeight;
    minX = minY = minHeight = 0;//= maxX = maxY = maxHeight = 0.0;
    std::transform(inputType.begin(), inputType.end(), inputType.begin(), ::tolower);
    if (inputType.compare("gdal") == 0) {
        // Setup all GDAL-supported raster drivers
        GDALAllRegister();

        // Open the input dataset
        if (verbose) cout << "Reading the input GDAL dataset..." << flush;
        GDALDataset *gdalDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
        if (gdalDataset == NULL) {
            cerr << "\n[Error] could not open the GDAL dataset" << endl;
            return 1;
        }

        // Copy the raster data into an array
        GDALRasterBand *heightsBand = gdalDataset->GetRasterBand(1);
        float *rasterHeights = new float[heightsBand->GetXSize() * heightsBand->GetYSize()];
        if (heightsBand->RasterIO(GF_Read, 0, 0, heightsBand->GetXSize(), heightsBand->GetYSize(),
                                  (void *) rasterHeights,
                                  heightsBand->GetXSize(), heightsBand->GetYSize(),
                                  GDT_Float32, 0, 0) != CE_None) {
            std::cerr << "\n[ERROR] Could not read heights from raster" << std::endl;
            return EXIT_FAILURE;
        }

        // Run over the array and create a
        minHeight =  std::numeric_limits<float>::infinity() ;
        //maxHeight = -std::numeric_limits<float>::infinity() ;
        for (int i = 0; i < heightsBand->GetXSize(); i++) {
            for (int j = 0; j < heightsBand->GetYSize(); j++) {
                int y = heightsBand->GetXSize() - 1 - j; // y coordinate within the tile.
                // Note that the heights in RasterIO have the origin in the upper-left corner,
                // while the tile has it in the lower-left. Obviously, x = i

                // Height value
                float height = rasterHeights[j * heightsBand->GetXSize() + i];

                // Clipping
                height = std::max(clippingLowValue, std::min(height, clippingHighValue));

                // When no data is available, we assume ground data
                if (height == heightsBand->GetNoDataValue())
                    height = 0;

                // If the input DEM contains bathymetry, consider the data as depth instead of altitude (negative value!)
                if (bathymetryFlag)
                    height = -height;

                if (height < minHeight)
                    minHeight = height;
                if (height > maxHeight)
                    maxHeight = height;

                // In heightmap format
                samples.push_back(Point_3(i, y, height));
            }
        }

        double width = gdalDataset->GetRasterXSize();
        double height = gdalDataset->GetRasterYSize();
        double gt[6];
        gdalDataset->GetGeoTransform(gt);
        minX = gt[0];
        minY = gt[3] + width*gt[4] + height*gt[5];
        maxX = gt[0] + width*gt[1] + height*gt[2];
        maxY = gt[3];

        // Delete the allocated memory that is not needed anymore
        delete rasterHeights;
        delete gdalDataset;
        if(verbose) cout << " done." << endl;
    }
    else if (inputType.compare("point-set") == 0)
    {
        if (verbose) cout << "Reading the input point set..." << flush;
        std::ifstream stream(inputFile.c_str());
        bool success = stream &&
                       CGAL::read_xyz_points(stream, std::back_inserter(samples));
        if (!success)
        {
            std::cerr << "\n[Error] Cannot read point set file " << inputFile << std::endl;
            return EXIT_FAILURE;
        }
        stream.close();
        if (verbose) cout << " done." << endl ;
    }
    else if (inputType.compare("off") == 0)
    {
        if (verbose) cout << "Reading the input OFF polyhedron..." << flush;
        Polyhedron poly ;
        std::ifstream stream(inputFile.c_str());
        stream >> poly;
        for (Polyhedron::Vertex_iterator it = poly.vertices_begin(); it != poly.vertices_end(); ++it )
            samples.push_back((*it).point());
        stream.close();
        if (verbose) cout << " done." << endl ;
    }
    else {
        std::cerr << "\n[ERROR] Unknown input type \"" << inputType << "\", available options are \"gdal\", \"point-set\" or \"off\"" << std::endl;
        return EXIT_FAILURE;
    }

    // Compute Bounding Box for inputs other than GDAL rasters
    if (inputType.compare("gdal") != 0) {
        K::Iso_cuboid_3 boundingBox = CGAL::bounding_box(samples.begin(), samples.end());
        minX = boundingBox.xmin();
        minY = boundingBox.ymin();
        minHeight = boundingBox.zmin();
        maxX = boundingBox.xmax();
        maxY = boundingBox.ymax();
        maxHeight = boundingBox.zmax();
    }

    for (std::vector<Point_3>::iterator it = samples.begin(); it != samples.end(); ++it) {
        double u = remap(it->x(), minX, maxX, 0.0, 1.0);
        double v = remap(it->y(), minY, maxY, 0.0, 1.0);
        double h = remap(it->z(), minHeight, maxHeight, 0.0, 1.0);

        *it = Point_3(u, v, h);
    }

    // Setup the TIN creator
    if(verbose) cout << "Creating the TIN..." << flush;
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
        if (greedyErrorType.compare("height") == 0) {
            et = TinCreationGreedyInsertionStrategy::ErrorHeight;
        }
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
//    else if (tinCreationStrategy.compare("remeshing") == 0) {
//        std::shared_ptr<TinCreationRemeshingStrategy> tcRemesh
//                = std::make_shared<TinCreationRemeshingStrategy>(remeshingFacetDistance,
//                                                                 remeshingFacetAngle,
//                                                                 remeshingFacetSize,
//                                                                 remeshingEdgeSize);
//        tinCreator.setCreator(tcRemesh);
//    }
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
        std::cerr << "\n[ERROR] Unknown TIN creation strategy \"" << tinCreationStrategy << "\"" << std::endl;
        return 1;
    }

    // Set the bounds of the "tile" if not using the remeshing or point set algorithm
    tinCreator.setBounds(minX, minY, minHeight, maxX, maxY, maxHeight);

    // Set parameters for zoom 0 (i.e., use the input parameters, since there will be no more zooms)
    tinCreator.setParamsForZoom(0);

    // Create the TIN and compute the time spent to do it
    auto start = std::chrono::high_resolution_clock::now();
    Polyhedron poly = tinCreator.create(samples);
    auto finish = std::chrono::high_resolution_clock::now();

    chrono::duration<double> elapsed = finish - start;
    if(verbose) cout << " done, " << elapsed.count() << " seconds." << endl ;

    // Recover the real values for the coordinates
    for (Polyhedron::Point_iterator it = poly.points_begin(); it != poly.points_end(); ++it) {
        double x = remap(it->x(), 0.0, 1.0, minX, maxX);
        double y = remap(it->y(), 0.0, 1.0, minY, maxY);
        double z = remap(it->z(), 0.0, 1.0, minHeight, maxHeight);

        *it = Point_3(x, y, z);
    }

    // Reset the origin of the samples for better stability during visualization in common 3D viewers
    if (resetOrigin) {
        if(verbose) cout << "Re-setting the origin of the samples..." << flush ;
        K::Iso_cuboid_3 boundingBox = CGAL::bounding_box(poly.points_begin(), poly.points_end());
        std::transform(poly.points_begin(), poly.points_end(), poly.points_begin(),
                       [boundingBox](Point_3& p){
                           return Point_3(p.x()-boundingBox.xmin(),
                                          p.y()-boundingBox.ymin(),
                                          p.z()-boundingBox.zmin());
                       }
        );
        if(verbose) cout << " done" << std::endl ;

        if (!exportNewOrigin.empty()) {
            if(verbose) cout << "Saving the new origin of the samples to file..." << flush ;
            ofstream ofOrig(exportNewOrigin);
            ofOrig << boundingBox.xmin() << std::endl
                   << boundingBox.ymin() << std::endl
                   << boundingBox.zmin() << std::endl;
            ofOrig.close();
            if(verbose) cout << " done." << endl ;
        }
    }
    else {
        if (!exportNewOrigin.empty()) {
            std::cerr << "[WARNING] The export-new-origin option should only be used when reset-origin is set to true (otherwise, the origin is not changed). We will ignore this parameter." << std::endl;
        }
    }

    // Save the results
    if(verbose) cout << "Saving the results..." << flush ;
    ofstream of(outputFile);
    of << poly;
    if(verbose) cout << " done." << endl ;

    return EXIT_SUCCESS;
}