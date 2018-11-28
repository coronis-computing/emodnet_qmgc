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

#ifndef EMODNET_QMGC_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
#define EMODNET_QMGC_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H

#include <ctb.hpp>
#include "tin_creation/tin_creation_cgal_types.h"
#include "quantized_mesh_tiler.h"
#include "zoom_tiles_scheduler.h"
#include "zoom_tiles_border_vertices_cache.h"
#include <iostream>
#include <vector>
#include <mutex>
#include "borders_data.h"



/**
 * @class QuantizedMeshTilesPyramidBuilder
 * @brief Contains the logic to create the pyramid of tiles in quantized mesh format.
 *
 * Since the quantized mesh format requires coherence between neighboring tiles, this class is the responsible
 * of maintaining this coherence and schedule the building of tiles by taking into account the ones already built.
 */
class QuantizedMeshTilesPyramidBuilder
{
    // --- Private typedefs ---
    typedef typename TinCreation::Point_3 Point_3;

public:

    /**
     * Constructor
     * @param qmTilers Vector of tilers, one for each desired thread (they should be the same!)
     * @param scheduler The desired scheduler defining a preferred order for processing the tiles
     */
    QuantizedMeshTilesPyramidBuilder(const std::vector<QuantizedMeshTiler>& qmTilers,
                                             const ZoomTilesScheduler& scheduler);

    /**
     * @brief Creates the tile pyramid in quantized-mesh format
     *
     * Due to the quantized-mesh format requiring the vertices on the edges to coincide between neighbors, the creation
     * of the tiles' for each zoom is not as simple as in the heightmap format, and it requires a more complex loop taking
     * into account vertices at borders of the neighbors of the current tile being processed
     */
    void createTmsPyramid(const int &startZoom, const int &endZoom, const std::string &outDir, const std::string &debugDir = std::string("")) ;

    /**
     * @brief Creates the tile pyramid in quantized-mesh format without taking into account coherence along neighboring tiles' borders
     * Useful when we know, by the way the simplification is created, that the vertices in the borders will always be the same (e.g., in the Delaunay TIN creation strategy)
     */
    void createTmsPyramidUnconstrainedBorders(const int &startZoom, const int &endZoom, const std::string &outDir, const std::string &debugDir = std::string("")) ;

    /**
     * @brief Check if the tile folder (zoom/x) exists, and creates it otherwise.
     *
     * @return The path of the tile's file.
     */
    static std::string getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                 const std::string &mainOutDir ) ;

    /**
     * @brief Create the required tile, in the given thread, and with the given data at the borders to maintain.
     * The result is an output tile file created, and an updated BorderData structure with the data to maintain for neighboring tiles to be created.
     * @param coord The tile coordinates
     * @param numThread The thread number where the process will take place
     * @param outDir The output directory where the output tile file will be generated
     * @param bd The BordersData structure input. It contains the data to preserve at the borders as a result of previously generated tiles.
     * @return It returns the data at the borders to be maintained for neighboring tiles to be generated in the future (i.e., what was already restricted in \p bd, plus the new borders created).
     */
    BordersData createTile( const ctb::TileCoordinate& coord,
                            const int& numThread,
                            const std::string& outDir,
                            const BordersData& bd ) ;

    /**
     * Get the next tile to process in the current zoom
     * @param tileXY The (x,y) coordinates of the tile to process within the current zoom
     * @return Returns true if a tile to be processed was located in the queue, false otherwise
     */
    bool getNextTileToProcess(ctb::TilePoint& tileXY) ;

private:
    // --- Attributes ---
    int m_numThreads ;
    std::vector<QuantizedMeshTiler> m_tilers ;
    ZoomTilesScheduler m_scheduler ;
    std::vector<ctb::TilePoint> m_tilesWaitingToProcess ;
    ZoomTilesBorderVerticesCache m_bordersCache ;
    bool m_debugMode ;
    std::string m_debugDir ;
    std::mutex m_diskWriteMutex;

    /**
    * @brief Check that the DEBUG tile folder (zoom/x) exists, and creates it otherwise.
    *
    * @return The path of the debug OFF tile's file.
    */
    std::string getDebugTileFileAndCreateDirs( const ctb::TileCoordinate &coord ) ;

};


#endif //EMODNET_QMGC_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
