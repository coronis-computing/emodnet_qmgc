//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

// Boost
#include <boost/program_options.hpp>
// Std
#include <iostream>
#include <fstream>
// JSON
#include <nlohmann/json.hpp>
// Cesium-terrain-builder
#include <ctb.hpp>
// Project-specific
#include "quantized_mesh_tile.h"

using namespace std ;
namespace po = boost::program_options ;
using json = nlohmann::json;


void compareVertices(const QuantizedMeshTile& qmt, const QuantizedMeshTile& qmtN,
                     const std::vector<unsigned int>& originalInds, const std::vector<unsigned int>& neighborInds,
                     double& meanDiffLon, double& meanDiffLat, double& meanDiffHeight)
{
    QuantizedMesh::VertexData vdO = qmt.getVertexData();
    QuantizedMesh::VertexData vdN = qmtN.getVertexData();
    std::vector<double> allLonO, allLatO, allHO, allLonN, allLatN, allHN;
    for (int i = 0; i < originalInds.size(); i++) {
        double lonO, latO, hO, lonN, latN, hN;
        qmt.convertUVHToLonLatHeight(vdO.u[originalInds[i]], vdO.v[originalInds[i]], vdO.height[originalInds[i]], lonO, latO, hO);
        qmtN.convertUVHToLonLatHeight(vdN.u[neighborInds[i]], vdN.v[neighborInds[i]], vdN.height[neighborInds[i]], lonN, latN, hN);

        allLonO.push_back(lonO);
        allLatO.push_back(latO);
        allHO.push_back(hO);
        allLonN.push_back(lonN);
        allLatN.push_back(latN);
        allHN.push_back(hN);
    }

    // Sort the arrays
    std::sort(allLonO.begin(), allLonO.end());
    std::sort(allLatO.begin(), allLatO.end());
    std::sort(allHO.begin(), allHO.end());
    std::sort(allLonN.begin(), allLonN.end());
    std::sort(allLatN.begin(), allLatN.end());
    std::sort(allHN.begin(), allHN.end());

    // Check equality
    meanDiffLon = 0;
    meanDiffLat = 0;
    meanDiffHeight = 0;
    int numVerts = allLonO.size();

    for ( int i = 0; i < numVerts; i++ ) {
        meanDiffLon += fabs(allLonO[i]-allLonN[i]);
        meanDiffLat += fabs(allLatO[i]-allLatN[i]);
        meanDiffHeight += fabs(allHO[i]-allHN[i]);
    }
    meanDiffLon /= numVerts;
    meanDiffLat /= numVerts;
    meanDiffHeight /= numVerts;

//    for (int i = 0; i < allLonO.size(); i++) {
//        cout << "lonO = " << allLonO[i] << endl;
//        cout << "lonN = " << allLonN[i] << endl;
//    }
//
//    for (int i = 0; i < allLatO.size(); i++) {
//        cout << "latO = " << allLatO[i] << endl;
//        cout << "latN = " << allLatN[i] << endl;
//    }
//
//    for (int i = 0; i < allHO.size(); i++) {
//        cout << "hO = " << allHO[i] << endl;
//        cout << "hN = " << allHN[i] << endl;
//    }
}

int main ( int argc, char **argv)
{
    // Parse input parameters
    std::string inputDir;
    po::options_description options("Check the consistency between neighboring tiles at borders");
    bool displayTileInfo, displayTileStats, displayZoomStats;
    options.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<std::string>(&inputDir), "Input directory containing the TMS structure of quantized mesh tiles generated with qm_tiler")
            ("display-tile-info", po::value<bool>(&displayTileInfo)->default_value(false), "Display additional information for each tile")
            ("display-tile-stats", po::value<bool>(&displayTileStats)->default_value(false), "Display the mean differences at the borders for each tile")
            ("display-zoom-stats", po::value<bool>(&displayZoomStats)->default_value(true), "Display the mean differences at the borders for each zoom")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Read the layer.json file
    std::ifstream ifs(inputDir + "/layer.json");
    if (!ifs) {
        cerr << "[ERROR] Cannot read layer.json file from folder " << inputDir << endl;
        return 1;
    }
    json j;
    ifs >> j;
    json jZooms = j["available"] ; // We're only interested in the zooms part

    ctb::GlobalGeodetic profile;
    double globalMeanDiffLon = 0, globalMeanDiffLat = 0, globalMeanDiffHeight = 0;
    for ( int zoom = 0; zoom < jZooms.size(); zoom++ ) {
        int startX = jZooms[zoom][0]["startX"].get<int>();
        int endX = jZooms[zoom][0]["endX"].get<int>();
        int startY = jZooms[zoom][0]["startY"].get<int>();
        int endY = jZooms[zoom][0]["endY"].get<int>();

        if (displayZoomStats) {
            cout << "----- Processing zoom " << zoom << " -----" << endl;
            cout << "startX = " << startX << endl;
            cout << "endX = " << endX << endl;
            cout << "startY = " << startY << endl;
            cout << "endY = " << endY << endl;
        }

        double zoomMeanDiffLon = 0, zoomMeanDiffLat = 0, zoomMeanDiffHeight = 0;
        int numTiles = (endX-startX+1)*(endY-startY+1);
        if (displayZoomStats) cout << "num tiles in zoom = " << numTiles << endl;
        // Visit each tile in the zoom
        for ( int x = startX; x <= endX; x++ ) {
            for ( int y = startY; y <= endY; y++ ) {
                // Load the tile
                std::ostringstream ss; ss << inputDir << "/" << zoom << "/" << x << "/" << y << ".terrain";
                const ctb::TileCoordinate coord(zoom, x, y);
                if (displayTileStats || displayTileInfo) cout << "Reading tile " << zoom << ", " << x << ", " << y << endl;
                QuantizedMeshTile qmt(coord);
                if ( !qmt.readFile(ss.str()) ) {
                    cerr << "[ERROR] Cannot read the input file" << endl ;
                    return 1;
                }
                QuantizedMesh::EdgeIndices eiO = qmt.getEdgeIndices();

                if (displayTileInfo) {
                    ctb::CRSBounds tileBounds = profile.tileBounds(qmt);
                    cout << "Tile Bounds:" << endl;
                    cout << "minX = " << tileBounds.getMinX() << endl;
                    cout << "maxX = " << tileBounds.getMaxX() << endl;
                    cout << "minY = " << tileBounds.getMinY() << endl;
                    cout << "maxY = " << tileBounds.getMaxY() << endl;
                    cout << "Tile num. vertices:" << endl;
                    cout << "num. vert. north = " << eiO.northVertexCount << endl;
                    cout << "num. vert. south = " << eiO.southVertexCount << endl;
                    cout << "num. vert. east = " << eiO.eastVertexCount << endl;
                    cout << "num. vert. west = " << eiO.westVertexCount << endl;
                }

                /* Load and compare with the 4 neighbors: */
                std::vector<std::pair<int, int>> neighs;
                neighs.emplace_back(std::pair<int, int>(x,y+1)); // Northern neighbor
                neighs.emplace_back(std::pair<int, int>(x,y-1)); // Southern neighbor
                neighs.emplace_back(std::pair<int, int>(x+1,y)); // Eastern neighbor
                neighs.emplace_back(std::pair<int, int>(x-1,y)); // Western neighbor
                std::vector<std::pair<int, int>>::iterator it; int pairInd = 0;
                for ( it = neighs.begin(); it != neighs.end(); ++it, pairInd++ ) {
                    // Check bounds
                    if ( it->first >= startX && it->first <= endX && it->second >= startY && it->second <= endY ) {
                        std::ostringstream ssN; ssN << inputDir << "/" << zoom << "/" << it->first << "/" << it->second << ".terrain";
                        if (displayTileStats || displayTileInfo) cout << "Reading neighboring tile " << zoom << ", " << it->first << ", " << it->second << endl << " from file " << ssN.str() << endl;
                        const ctb::TileCoordinate coord(zoom, it->first, it->second);
                        QuantizedMeshTile qmtN(coord);
                        if (!qmtN.readFile(ssN.str())) {
                            cerr << "[ERROR] Cannot read the input file" << endl;
                            return 1;
                        }
                        QuantizedMesh::EdgeIndices eiN = qmtN.getEdgeIndices();
                        QuantizedMesh::VertexData vdN = qmtN.getVertexData();

                        if (displayTileInfo) {
                            ctb::CRSBounds tileBoundsN = profile.tileBounds(qmtN);
                            cout << "Neighboring Tile Bounds:" << endl;
                            cout << "minX = " << tileBoundsN.getMinX() << endl;
                            cout << "maxX = " << tileBoundsN.getMaxX() << endl;
                            cout << "minY = " << tileBoundsN.getMinY() << endl;
                            cout << "maxY = " << tileBoundsN.getMaxY() << endl;
                            cout << "num. vert. north = " << eiN.northVertexCount << endl;
                            cout << "num. vert. south = " << eiN.southVertexCount << endl;
                            cout << "num. vert. east = " << eiN.eastVertexCount << endl;
                            cout << "num. vert. west = " << eiN.westVertexCount << endl;
                        }

                        double meanDiffLon = 0, meanDiffLat = 0, meanDiffHeight = 0;
                        if (pairInd == 0) {
                            // Tiles vertices should coincide at northern (original) --> southern (neighbor) border
                            if (eiO.northVertexCount != eiN.southVertexCount ) {
                                cout << "[ERROR] Number of vertices does not coincide for northern neighbor!" << endl;
                                cout << "current tile northVertexCount = " << eiO.northVertexCount << " / neighbor tile southVertexCount = " << eiN.southVertexCount << endl;
                                return 1;
                            }
                            if (displayTileStats) cout << "Comparing northern (original) --> southern (neighbor) border:" << endl;
                            compareVertices(qmt, qmtN, eiO.northIndices, eiN.southIndices, meanDiffLon, meanDiffLat, meanDiffHeight);
                        }
                        else if (pairInd == 1) {
                            // Tiles vertices should coincide at southern (original) --> northern (neighbor) border
                            if (eiO.southVertexCount != eiN.northVertexCount ) {
                                cout << "[ERROR] Number of vertices does not coincide for southern neighbor!" << endl;
                                cout << "current tile southVertexCount = " << eiO.southVertexCount << " / neighbor tile northVertexCount = " << eiN.northVertexCount << endl;
                                return 1;
                            }
                            if (displayTileStats) cout << "Comparing southern (original) --> northern (neighbor) border:" << endl;
                            compareVertices(qmt, qmtN, eiO.southIndices, eiN.northIndices, meanDiffLon, meanDiffLat, meanDiffHeight);
                        }
                        else if (pairInd == 2) {
                            // Tiles vertices should coincide at eastern (original) --> western (neighbor) border
                            if (eiO.eastVertexCount != eiN.westVertexCount ) {
                                cout << "[ERROR] Number of vertices does not coincide for eastern neighbor!" << endl;
                                cout << "current tile eastVertexCount = " << eiO.eastVertexCount << " / neighbor tile westVertexCount = " << eiN.westVertexCount << endl;
                                return 1;
                            }
                            if (displayTileStats) cout << "Comparing eastern (original) --> western (neighbor) border:" << endl;
                            compareVertices(qmt, qmtN, eiO.eastIndices, eiN.westIndices, meanDiffLon, meanDiffLat, meanDiffHeight);
                        }
                        else if (pairInd == 3) {
                            // Tiles vertices should coincide at western (original) --> eastern (neighbor) border
                            if (eiO.westVertexCount != eiN.eastVertexCount ) {
                                cout << "[ERROR] Number of vertices does not coincide for western neighbor!" << endl;
                                cout << "current tile westVertexCount = " << eiO.westVertexCount << " / neighbor tile eastVertexCount = " << eiN.eastVertexCount << endl;
                                return 1;
                            }
                            if (displayTileStats) cout << "Comparing western (original) --> eastern (neighbor) border:" << endl;
                            compareVertices(qmt, qmtN, eiO.westIndices, eiN.eastIndices, meanDiffLon, meanDiffLat, meanDiffHeight);
                        }
                        if (displayTileStats) {
                            cout << "Mean Diff. Lon. = " << meanDiffLon << endl;
                            cout << "Mean Diff. Lat. = " << meanDiffLat << endl;
                            cout << "Mean Diff. Height = " << meanDiffHeight << endl;
                        }
                        zoomMeanDiffLon += meanDiffLon;
                        zoomMeanDiffLat += meanDiffLat;
                        zoomMeanDiffHeight += meanDiffHeight;
                    }
                }
            }
        }
        zoomMeanDiffLon /= numTiles;
        zoomMeanDiffLat /= numTiles;
        zoomMeanDiffHeight /= numTiles;

        if (displayZoomStats) {
            cout << "----- End of zoom " << zoom << " -----" << endl;
            cout << "Mean Diff. Lon. = " << zoomMeanDiffLon << endl;
            cout << "Mean Diff. Lat. = " << zoomMeanDiffLat << endl;
            cout << "Mean Diff. Height = " << zoomMeanDiffHeight << endl;
        }

        globalMeanDiffLon += zoomMeanDiffLon;
        globalMeanDiffLat += zoomMeanDiffLat;
        globalMeanDiffHeight += zoomMeanDiffHeight;
    }

    globalMeanDiffLon /= jZooms.size();
    globalMeanDiffLat /= jZooms.size();
    globalMeanDiffHeight /= jZooms.size();

    cout << "----- Global statistics -----" << endl;
    cout << "Mean Diff. Lon. = " << globalMeanDiffLon << endl;
    cout << "Mean Diff. Lat. = " << globalMeanDiffLat << endl;
    cout << "Mean Diff. Height = " << globalMeanDiffHeight << endl;

    return 0;
}