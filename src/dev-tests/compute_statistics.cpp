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
 * @brief Computes some statistics from the tiles
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// Std
#include <iostream>
#include <fstream>
#include <map>
// JSON
#include <nlohmann/json.hpp>
// Cesium-terrain-builder
#include <ctb.hpp>
// Project-specific
#include "quantized_mesh_tile.h"
#include "quantized_mesh_tiler.h"
#include "tin_creation/tin_creator.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal/surface_mesh_from_projected_triangulation.h"
#include "base/crs_conversions.h"
// CGAL
#include <CGAL/Surface_mesh.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
// Boost
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>

#if defined(CGAL_LINKED_WITH_TBB)
#define TAG CGAL::Parallel_tag
#else
#define TAG CGAL::Sequential_tag
#endif

using namespace std;
using namespace TinCreation;
namespace po = boost::program_options;
using json = nlohmann::json;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef Gt::Rp::Triangle_3                                          Triangle_3;
typedef Gt::Rp::Line_3                                              Line_3;
typedef Gt::Rp::Segment_3                                           Segment_3;
typedef Gt::Rp::Intersect_3                                         Intersect_3;

typedef CGAL::Surface_mesh<Point_3> SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor edge_descriptor;



// Debug function
void exportToXYZ(const std::string& outFilePath, const std::vector<Point_3>& pts) {
    std::ofstream of( outFilePath.c_str(), std::ios_base::out ) ;
    if (!of.is_open()) {
        std::cout << "[ERROR] Cannot open XYZ file for writing" << std::endl;
        return;
    }

    for (std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it)
        of << (*it).x() << " " << (*it).y() << " " << (*it).z() << endl;

    of.close();
}



void computeErrorsInHeight(const QuantizedMeshTile& qmt, const Delaunay& dtRaster,
                           double& meanDist, double& maxDist )
{
// Create the mesh from the qm tile
    std::vector<Point_3> qmPts;
    QuantizedMeshTile::VertexData vertexData = qmt.getVertexData();
    QuantizedMeshTile::Header header = qmt.getHeader();
    for ( int i = 0; i < vertexData.vertexCount; i++ ) {
        // We want points in uv in the XY plane, but with real values in height
        double u = (double)vertexData.u[i]/(double)QuantizedMesh::MAX_VERTEX_DATA;
        double v = (double)vertexData.v[i]/(double)QuantizedMesh::MAX_VERTEX_DATA;
        double height = header.MinimumHeight + fabs( header.MaximumHeight - header.MinimumHeight ) * (double)vertexData.height[i]/(double)QuantizedMesh::MAX_VERTEX_DATA;
        qmPts.emplace_back(Point_3(u,v,height));
//                    std::cout << u << " " << v << " " << height << std::endl;
    }
    // Debug
//                exportToXYZ("pts_from_tile.xyz", qmPts);



    // Compute distances from points in the tile to the original raster values
    int numDists = 0;
    double dist = 0.0;
    meanDist = 0.0;
    maxDist = 0.0;
    for (std::vector<Point_3>::iterator it = qmPts.begin(); it != qmPts.end(); ++it) {
        // Locate the point in the triangulation
        Delaunay::Face_handle fh = dtRaster.locate(*it);
        Triangle_3 t = dtRaster.triangle(fh);

        // Construct a line in the Z direction
        Line_3 l(*it, Vector_3(0, 0, 1));

        // Compute their intersection
        CGAL::cpp11::result_of<Intersect_3(K::Line_3, K::Triangle_3)>::type intersect = CGAL::intersection(l, t);

        // If everything goes as expected, the intersection should exist and should be a point
        if (!intersect) {
            std::cerr << "Error! Empty intersection" << std::endl;
            dist = 0;
            continue;
        }
        if (const Segment_3 *s = boost::get<Segment_3>(&*intersect)) {
            std::cerr << "Error! Segment intersection" << std::endl;
            dist = 0;
            continue;
        }

        // Get the intersection point
        const Point_3 *ip = boost::get<Point_3>(&*intersect);

        // Finally, compute the squared distance between the query point and the intersection
        dist = CGAL::squared_distance(*it, *ip);
        dist = CGAL::sqrt(dist);
//                    std::cout << "dist = " << dist << std::endl;

        // Update running mean
        numDists++;
        meanDist = (meanDist*(numDists-1)+dist) / numDists;

        if (dist > maxDist) {
            maxDist = dist;
        }
    }
}



void computeErrorsInECEF(const QuantizedMeshTile& qmt, const Delaunay& dtRaster, const ctb::CRSBounds& tileBounds,
                         double& hausdorffDistRasterToTin, double& hausdorffDistTinToRaster)
{
    // --- Compute distances in a metric space ---
    // For that, create meshes with the same connectivity as before, but with ECEF vertices

    QuantizedMeshTile::VertexData vertexData = qmt.getVertexData();

    // --> Raster mesh
    // Translate to a surface mesh
    SurfaceMesh meshRaster = surfaceMeshFromProjectedTriangulation<Delaunay, SurfaceMesh>(dtRaster);
    // Transform vertices to ECEF (preserve connectivity)
    std::vector<Point_3> ptsRaster;
    BOOST_FOREACH(vertex_descriptor vd, vertices(meshRaster)){
                    Point_3 p = meshRaster.point(vd);
                    // From UV to lat/lon (height already in the correct units)
                    double lat = tileBounds.getMinY() + fabs(tileBounds.getMaxY() - tileBounds.getMinY()) * p.y();
                    double lon = tileBounds.getMinX() + fabs(tileBounds.getMaxX() - tileBounds.getMinX()) * p.x();

                    double tmpx, tmpy, tmpz;
                    crs_conversions::llh2ecef(lat, lon, p.z(), tmpx, tmpy, tmpz);

                    meshRaster.point(vd) = Point_3(tmpx, tmpy, tmpz);
                    ptsRaster.push_back(Point_3(tmpx, tmpy, tmpz));
                }
    ofstream ofMeshRaster("mesh_raster.off");
    ofMeshRaster << meshRaster;
    ofMeshRaster.close();

    // --> TIN mesh
    SurfaceMesh meshTin;
    std::map<int, SurfaceMesh::Vertex_index> indToVertIndMap;
    for ( int i = 0; i < vertexData.vertexCount; i++ ) {
        double lat, lon, height;
        qmt.convertUVHToLonLatHeight(vertexData.u[i], vertexData.v[i], vertexData.height[i], lon, lat, height );
        double tmpx, tmpy, tmpz;
        crs_conversions::llh2ecef(lat, lon, height, tmpx, tmpy, tmpz);
        SurfaceMesh::Vertex_index vi = meshTin.add_vertex(Point_3(tmpx, tmpy, tmpz));
        indToVertIndMap.insert(std::pair<int, SurfaceMesh::Vertex_index>(i, vi));
    }
    QuantizedMeshTile::IndexData triIndices = qmt.getIndexData();
    for ( int i = 0; i < triIndices.triangleCount; i++ ) {
        meshTin.add_face(indToVertIndMap[triIndices.indices[3*i]],
                         indToVertIndMap[triIndices.indices[(3*i)+1]],
                         indToVertIndMap[triIndices.indices[(3*i)+2]]);
    }
    ofstream ofMeshTin("mesh_tin.off");
    ofMeshTin << meshTin;
    ofMeshTin.close();

    // Bidirectional Hausdorff distance computation
    hausdorffDistRasterToTin = PMP::approximate_Hausdorff_distance<TAG>(meshRaster, meshTin,
                                                                        PMP::parameters::all_default());

    hausdorffDistTinToRaster = PMP::approximate_Hausdorff_distance<TAG>(meshTin, meshRaster,
                                                                        PMP::parameters::number_of_points_on_faces(meshRaster.number_of_vertices()));
}



template<class T>
void writeInMatlabFormat(ofstream& ofs, const std::string& varName, const std::vector<T>& vect)
{
    ofs << varName << " = [" << endl;
    for(int i=0; i < vect.size(); i++) {
        ofs << vect[i];
        if (i == vect.size()-1)
            ofs << "];" << endl;
        else
            ofs << ";" << endl;
    }
}

bool exportToMatlab(const std::string& outFile,
                    const std::vector<int>& meanNumVertPerTile,
                    const std::vector<int>& meanNumTrianglesPerTile,
                    const std::vector<double>& meanHeightErrorPerTile,
                    const std::vector<double>& maxHeightErrorPerTile,
                    const std::vector<double>& hausdorffDistRasterToTinPerTile,
                    const std::vector<double>& hausdorffDistTinToRasterPerTile,
                    const std::vector<double>& symmetricHausdorffDistancePerTile,
                    const double& numVertRasterTile,
                    const double& numTrianglesRasterTile,
                    const double& meanNumVert,
                    const double& meanNumTriangles,
                    const double& meanHeightError,
                    const double& maxHeightError,
                    const double& meanSymmetricHausdorffDistance)
{
    ofstream ofs(outFile);
    if (!ofs) {
        cerr << "[ERROR] Cannot open output file!" << endl;
        return false;
    }

    writeInMatlabFormat(ofs, "meanNumVertPerTile", meanNumVertPerTile);
    writeInMatlabFormat(ofs, "meanNumTrianglesPerTile", meanNumTrianglesPerTile);
    writeInMatlabFormat(ofs, "meanHeightErrorPerTile", meanHeightErrorPerTile);
    writeInMatlabFormat(ofs, "maxHeightErrorPerTile", maxHeightErrorPerTile);
    writeInMatlabFormat(ofs, "hausdorffDistRasterToTinPerTile", hausdorffDistRasterToTinPerTile);
    writeInMatlabFormat(ofs, "hausdorffDistTinToRasterPerTile", hausdorffDistTinToRasterPerTile);
    writeInMatlabFormat(ofs, "symmetricHausdorffDistancePerTile", symmetricHausdorffDistancePerTile);

    ofs << "numVertRasterTile = " << numVertRasterTile << ";" << endl;
    ofs << "numTrianglesRasterTile = " << numTrianglesRasterTile << ";" << endl;
    ofs << "meanNumVert = " << meanNumVert << ";" << endl;
    ofs << "meanNumTriangles = " << meanNumTriangles << ";" << endl;
    ofs << "meanHeightError = " << meanHeightError << ";" << endl;
    ofs << "maxHeightError = " << maxHeightError << ";" << endl;
    ofs << "meanSymmetricHausdorffDistance = " << meanSymmetricHausdorffDistance << ";" << endl;

    return true;
}



int main(int argc, char **argv)
{
    // Parse input parameters
    std::string tilesInputDir, inputRaster, configFile, outMatlabFile;
    int heighMapSamplingSteps, zoom;
    float clippingHighValue, clippingLowValue, aboveSeaLevelScaleFactor, belowSeaLevelScaleFactor;
    bool bathymetryFlag, verbose;
    po::options_description options("Options:");
    options.add_options()
            ("help,h", "Produce help message")
            ("input-tiles,t", po::value<std::string>(&tilesInputDir), "Input directory containing the TMS structure of quantized mesh tiles generated with qm_tiler")
            ("input-raster,i", po::value<std::string>(&inputRaster), "Input raster from which we generated the TMS structure pointed by --input-tiles")
            ("zoom,z", po::value<int>(&zoom)->default_value(0), "Zoom level of the pyramid to extract the statistics from" )
            ("output,o", po::value<std::string>(&outMatlabFile)->default_value("out_stats_zoom.m"), "Output matlab file containing the per-tile results, along with the global statistics computed in this function")
            ("bathymetry,b", po::value<bool>(&bathymetryFlag)->default_value(false), "Switch to consider the input DEM as containing depths instead of elevations" )
            ("samples-per-tile", po::value<int>(&heighMapSamplingSteps)->default_value(256), "Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that the statistics will be computed with respect to this raster resolution." )
            ("clip-high", po::value<float>(&clippingHighValue)->default_value(std::numeric_limits<float>::infinity()), "Clip values in the DEM above this threshold." )
            ("clip-low", po::value<float>(&clippingLowValue)->default_value(-std::numeric_limits<float>::infinity()), "Clip values in the DEM below this threshold." )
            ("above-sea-level-scale-factor", po::value<float>(&aboveSeaLevelScaleFactor)->default_value(-1), "Scale factor to apply to the readings above sea level (ignored if < 0)" )
            ("below-sea-level-scale-factor", po::value<float>(&belowSeaLevelScaleFactor)->default_value(-1), "Scale factor to apply to the readings below sea level (ignored if < 0)" )
            ("config,c", po::value<string>(&configFile)->default_value(""), "Configuration file with a set of the options above specified in the form <option>=<value>. Note that the options in the config file have preference over the ones specified on the command line.")
            ("verbose", po::value<bool>(&verbose)->default_value(true), "Activate/deactivate output on the screen")
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
        cout << "Extracts some statistics from the data"
             << options << "\n";
        return 1;
    }

    // Read the layer.json file
    std::ifstream ifs(tilesInputDir + "/layer.json");
    if (!ifs) {
        cerr << "[ERROR] Cannot read layer.json file from folder " << tilesInputDir << endl;
        return 1;
    }
    json j;
    ifs >> j;
    json jZooms = j["available"] ; // We're only interested in the zooms part

    // Setup the tiler
    GDALAllRegister(); // Setup all GDAL-supported raster drivers
    GDALDataset *gdal = (GDALDataset *) GDALOpen(inputRaster.c_str(), GA_ReadOnly);
    if (gdal == NULL) {
        cerr << "[Error] Could not open GDAL dataset" << endl;
        return 1;
    }

    const char *proj = gdal->GetProjectionRef();
    OGRSpatialReference oSRS(proj);
    if (!oSRS.IsGeographic() || strcmp(oSRS.GetAttrValue("geogcs"), "WGS 84") != 0) {
        cerr << "[ERROR] We require the input dataset to be using the WGS 84 reference system."
             << endl;
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

    TinCreator tinCreator; // Empty TinCreator, we don't need it

    QuantizedMeshTiler tiler(gdal, grid, gdalTilerOptions, qmtOptions, tinCreator);

    // Run over zooms/tiles
    ctb::GlobalGeodetic profile;
    double globalMeanDiffLon = 0, globalMeanDiffLat = 0, globalMeanDiffHeight = 0;

    // Compute limits of the zoom
    int startX = jZooms[zoom][0]["startX"].get<int>();
    int endX = jZooms[zoom][0]["endX"].get<int>();
    int startY = jZooms[zoom][0]["startY"].get<int>();
    int endY = jZooms[zoom][0]["endY"].get<int>();

    if (verbose) {
        cout << "----- Processing zoom " << zoom << " -----" << endl;
        cout << "startX = " << startX << endl;
        cout << "endX = " << endX << endl;
        cout << "startY = " << startY << endl;
        cout << "endY = " << endY << endl;
    }

    // Total number of tiles
    int numTiles = (endX - startX + 1) * (endY - startY + 1);
    if (verbose) cout << "num tiles in zoom = " << numTiles << endl;

    // Variables needed to compute the stats
    int curNumTiles=0;
    double meanNumVert = .0;
    double meanNumTriangles = .0;
    double meanHeightError = .0;
    double maxHeightError = .0;
    double meanHausdorffDistRasterToTin = .0;
    double meanHausdorffDistTinToRaster = .0;
    double meanSymmetricHausdorffDistance = .0;
    bool computeOnce = false;
    int numVertRasterTile = 0;
    int numTrianglesRasterTile = 0;
    // Variables to store the statistics per tile
    std::vector<int> meanNumVertPerTile; meanNumVertPerTile.reserve(numTiles);
    std::vector<int> meanNumTrianglesPerTile; meanNumTrianglesPerTile.reserve(numTiles);
    std::vector<double> meanHeightErrorPerTile; meanHeightErrorPerTile.reserve(numTiles);
    std::vector<double> maxHeightErrorPerTile; maxHeightErrorPerTile.reserve(numTiles);
    std::vector<double> hausdorffDistRasterToTinPerTile; hausdorffDistRasterToTinPerTile.reserve(numTiles);
    std::vector<double> hausdorffDistTinToRasterPerTile; hausdorffDistTinToRasterPerTile.reserve(numTiles);
    std::vector<double> symmetricHausdorffDistancePerTile; symmetricHausdorffDistancePerTile.reserve(numTiles);

    // Visit each tile in the zoom
    int numDispDigits = ceil(log10(numTiles));
    for (int x = startX; x <= endX; x++) {
        for (int y = startY; y <= endY; y++) {

//                cout << "\r- Processing tile " << setw(numDispDigits) << curNumTiles << "/" << numTiles << flush;
            cout << "- Processing tile " << setw(numDispDigits) << curNumTiles << "/" << numTiles << endl;

            // Load the tile
            std::ostringstream ss;
            ss << tilesInputDir << "/" << zoom << "/" << x << "/" << y << ".terrain";
            const ctb::TileCoordinate coord(zoom, x, y);
            QuantizedMeshTile qmt(coord);
            if (!qmt.readFile(ss.str())) {
                cerr << "[ERROR] Cannot read the input file" << endl;
                return 1;
            }

            // Create the mesh from the raster tile
            float minHeight, maxHeight;
            ctb::CRSBounds tileBounds;
            BordersData bd = BordersData();
            std::vector<Point_3> rasterPts = tiler.getUVHPointsFromRaster(coord, bd, minHeight, maxHeight, tileBounds);

            // We want points in uv in the XY plane, but with real values in height
            for (std::vector<Point_3>::iterator it = rasterPts.begin(); it != rasterPts.end(); ++it) {
                double height = minHeight + fabs(maxHeight - minHeight) * (*it).z();
                *it = Point_3((*it).x(), (*it).y(), height);
            }
            // Debug
//                exportToXYZ("pts_from_raster.xyz", rasterPts);
            Delaunay dtRaster(rasterPts.begin(), rasterPts.end());

            if (!computeOnce) {
                numVertRasterTile = rasterPts.size();
                numTrianglesRasterTile = dtRaster.number_of_faces();
                computeOnce = true;
            }

            // Compute errors in height only
            double meanHeightErrorInTile, maxHeightErrorInTile;
            computeErrorsInHeight(qmt, dtRaster, meanHeightErrorInTile, maxHeightErrorInTile);

            double hausdorffDistRasterToTin, hausdorffDistTinToRaster;
            computeErrorsInECEF(qmt, dtRaster, tileBounds, hausdorffDistRasterToTin, hausdorffDistTinToRaster);

            // Bidirectional Hausdorff distance: maximum of Hausdorff in both directions
            double hausdorffDist = std::max(hausdorffDistRasterToTin, hausdorffDistTinToRaster);

            // --- Update Statistics ---
            curNumTiles++;
            // Compute running average on number of vertices per tile
            meanNumVert = (meanNumVert*(curNumTiles-1)+qmt.getVertexData().vertexCount) / curNumTiles;
            // Compute running average on number of triangles per tile
            meanNumTriangles = (meanNumTriangles*(curNumTiles-1)+qmt.getIndexData().triangleCount) / curNumTiles;
            // Compute running average of mean error in height per tile
            meanHeightError = (meanHeightError*(curNumTiles-1)+meanHeightErrorInTile) / curNumTiles;
            // Compute running average of Hausdorff distances
            meanHausdorffDistRasterToTin = (meanHausdorffDistRasterToTin*(curNumTiles-1)+hausdorffDistRasterToTin) / curNumTiles;
            meanHausdorffDistTinToRaster = (meanHausdorffDistTinToRaster*(curNumTiles-1)+hausdorffDistTinToRaster) / curNumTiles;
            meanSymmetricHausdorffDistance = (meanSymmetricHausdorffDistance*(curNumTiles-1)+hausdorffDist) / curNumTiles;
            // Compute the maximum deviation for this zoom
            if (maxHeightErrorInTile > maxHeightError)
                maxHeightError = maxHeightErrorInTile;

            // --- Update variables storing per tile statistics ---
            meanNumVertPerTile.push_back(qmt.getVertexData().vertexCount);
            meanNumTrianglesPerTile.push_back(qmt.getIndexData().triangleCount);
            meanHeightErrorPerTile.push_back(meanHeightErrorInTile);
            maxHeightErrorPerTile.push_back(maxHeightErrorInTile);
            hausdorffDistRasterToTinPerTile.push_back(hausdorffDistRasterToTin);
            hausdorffDistTinToRasterPerTile.push_back(hausdorffDistTinToRaster);
            symmetricHausdorffDistancePerTile.push_back(hausdorffDist);
        }
    }

    cout << "\n----- Statistics for tiles in zoom " << zoom << " -----" << endl;
    cout << "From raster:" << endl;
    cout << " - Num. vertices per tile = " << numVertRasterTile << endl;
    cout << " - Num. triangles per tile = " << numTrianglesRasterTile << endl;
    cout << "From the TINs:" << endl;
    cout << " - Mean num. vertices = " << meanNumVert << endl;
    cout << " - Mean num. triangles = " << meanNumTriangles << endl;
    cout << " - Mean height error = " << meanHeightError << endl;
    cout << " - Max. height error = " << maxHeightError << endl;
    cout << " - Mean symmetric Hausdorff distance = " << meanSymmetricHausdorffDistance << endl;

    bool res = exportToMatlab(outMatlabFile,
                              meanNumVertPerTile,
                              meanNumTrianglesPerTile,
                              meanHeightErrorPerTile,
                              maxHeightErrorPerTile,
                              hausdorffDistRasterToTinPerTile,
                              hausdorffDistTinToRasterPerTile,
                              symmetricHausdorffDistancePerTile,
                              numVertRasterTile,
                              numTrianglesRasterTile,
                              meanNumVert,
                              meanNumTriangles,
                              meanHeightError,
                              maxHeightError,
                              meanSymmetricHausdorffDistance);
    if (!res) {
        cerr << "[ERROR] Problems writing the output matlab file" << endl;
        return EXIT_FAILURE;
    }

    delete (gdal);

    return EXIT_SUCCESS;
}