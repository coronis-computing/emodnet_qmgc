/**
 * @file
 * @brief Computes some statistics from the tiles
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// Boost
#include <boost/program_options.hpp>
// Std
#include <iostream>
#include <fstream>
// JSON
#include <nlohmann/json.hpp>
// Cesium-terrain-builder
#include <ctb.hpp>
// CGAL
#include <CGAL/intersections.h>
// Project-specific
#include "quantized_mesh_tile.h"
#include "quantized_mesh_tiler.h"
#include "tin_creation/tin_creator.h"
#include "tin_creation/tin_creation_cgal_types.h"


using namespace std;
using namespace TinCreation;
namespace po = boost::program_options;
using json = nlohmann::json;

typedef Gt::Rp::Triangle_3                                          Triangle_3;
typedef Gt::Rp::Line_3                                              Line_3;
typedef Gt::Rp::Segment_3                                           Segment_3;
typedef Gt::Rp::Intersect_3                                         Intersect_3;


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

int main ( int argc, char **argv)
{
    // Parse input parameters
    std::string tilesInputDir, inputRaster, configFile;
    int heighMapSamplingSteps, zoom;
    float clippingHighValue, clippingLowValue, aboveSeaLevelScaleFactor, belowSeaLevelScaleFactor;
    bool bathymetryFlag, verbose;
    po::options_description options("Options:");
    options.add_options()
            ("help,h", "Produce help message")
            ("input-tiles,t", po::value<std::string>(&tilesInputDir), "Input directory containing the TMS structure of quantized mesh tiles generated with qm_tiler")
            ("input-raster,i", po::value<std::string>(&inputRaster), "Input raster from which we generated the TMS structure pointed by --input-tiles")
            ("zoom,z", po::value<int>(&zoom)->default_value(0), "Zoom level of the pyramid to extract the statistics from" )
            ("bathymetry,b", po::value<bool>(&bathymetryFlag)->default_value(false), "Switch to consider the input DEM as containing depths instead of elevations" )
            ("samples-per-tile", po::value<int>(&heighMapSamplingSteps)->default_value(256), "Samples to take in each dimension per tile. While TMS tiles are supposed to comprise 256x256 pixels/samples, using this option we can sub-sample it to lower resolutions. Note that a smaller sampling provides a coarser base mesh that will be easier to simplify." )
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
//    for ( int zoom = 0; zoom < jZooms.size(); zoom++ ) {
//    for ( int zoom = jZooms.size()-1; zoom >=0; zoom-- ) {
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
        double meanNumVertPerTile = .0;
        double meanNumTrianglesPerTile = .0;
        double meanHeightErrorPerTile = .0;
        double maxHeightErrorPerTile = .0;

        // Visit each tile in the zoom
        for (int x = startX; x <= endX; x++) {
            for (int y = startY; y <= endY; y++) {
                // Load the tile
                std::ostringstream ss;
                ss << tilesInputDir << "/" << zoom << "/" << x << "/" << y << ".terrain";
                const ctb::TileCoordinate coord(zoom, x, y);
                QuantizedMeshTile qmt(coord);
                if (!qmt.readFile(ss.str())) {
                    cerr << "[ERROR] Cannot read the input file" << endl;
                    return 1;
                }

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

                // Compute distances from points in the tile to the original raster values
                int numDists = 0;
                double dist = 0.0;
                double meanDist = 0.0;
                double maxDist = 0.0;
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
                        std::cout << "NEW MAX: p = " << *it << ", ip = " << *ip << " = " << dist << std::endl;
                        maxDist = dist;
                    }
                }

                // --- Compute Statistics ---
                curNumTiles++;
                // Compute running average on number of vertices per tile
                meanNumVertPerTile = (meanNumVertPerTile*(curNumTiles-1)+qmt.getVertexData().vertexCount) / curNumTiles;
                // Compute running average on number of triangles per tile
                meanNumTrianglesPerTile = (meanNumTrianglesPerTile*(curNumTiles-1)+qmt.getIndexData().triangleCount) / curNumTiles;
                // Compute running average of mean error in height per tile
                meanHeightErrorPerTile = (meanHeightErrorPerTile*(curNumTiles-1)+meanDist) / curNumTiles;
                // Compute the maximum deviation for this zoom
                if (maxDist > maxHeightErrorPerTile)
                    maxHeightErrorPerTile = maxDist;

                cout << "meanHeightErrorPerTile = " << meanHeightErrorPerTile << endl;
                cout << "maxHeightErrorPerTile = " << maxHeightErrorPerTile << endl;
            }
        }

        cout << "----- Statistics for zoom " << zoom << " -----" << endl;
        cout << "meanNumVertPerTile = " << meanNumVertPerTile << endl;
        cout << "meanNumTrianglesPerTile = " << meanNumTrianglesPerTile << endl;
        cout << "meanHeightErrorPerTile = " << meanHeightErrorPerTile << endl;
//    }

    return 0;
}