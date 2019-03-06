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

#ifndef EMODNET_QMGC_ZOOM_TILES_BORDER_VERTICES_CACHE_H
#define EMODNET_QMGC_ZOOM_TILES_BORDER_VERTICES_CACHE_H

#include <ctb.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "tile_border_vertices.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include "borders_data.h"
#include <chrono>
#include <thread>

/**
 * @class ZoomTilesBorderVerticesCache
 * @brief Cache to store/reuse the vertices at the borders for tiles that have been already constructed for a given zoom.
 *
 * Once the information of a tile is no longer required, it is erased from this cache.
 */
class ZoomTilesBorderVerticesCache
{
    // --- Private typedefs ---
    typedef TinCreation::Point_3 Point_3;

public:
    /**
     * Constructor
     * @param zoomBounds The bounds of the current zoom
     * @param tileMaxCoord The number of pixels in a tile
     */
    ZoomTilesBorderVerticesCache( const ctb::TileBounds& zoomBounds, const int& tileMaxCoord)
            : m_mapTileToBorderVertices()
            , m_zoomBounds(zoomBounds)
            , m_tileMaxCoord( tileMaxCoord )
            , m_numProcessedTiles(0)
    {
        m_numTiles = (zoomBounds.getMaxY()-zoomBounds.getMinY()+1)*(zoomBounds.getMaxX()-zoomBounds.getMinX()+1) ;

        // Initialize vectors of bools
        int nx = zoomBounds.getMaxX()-zoomBounds.getMinX()+1; // Number of tiles in X
        int ny = zoomBounds.getMaxY()-zoomBounds.getMinY()+1; // Number of tiles in Y
        m_tilesVisited.resize(nx);
        for (int i = 0; i < nx; i++)
            m_tilesVisited[i].resize(ny, false);
        m_tilesBeingProcessed.resize(nx);
        for (int i = 0; i < nx; i++)
            m_tilesBeingProcessed[i].resize(ny, false);
    }

    /**
     * Default Constructor
     */
    ZoomTilesBorderVerticesCache()
            : m_mapTileToBorderVertices()
            , m_zoomBounds()
            , m_tileMaxCoord(0)
            , m_numProcessedTiles(0)
            , m_numTiles(0)
            , m_tilesVisited()
            , m_tilesBeingProcessed() {}

    /**
     * Given a tile, it gets the border edges to preserve from already constructed tiles.
     * It also cleans up unnecessary data and marks the tile as "being processed", so that neighboring tiles cannot start
     * processing until this one has finished.
     *
     * @param tileX The X coordinate of the tile to construct
     * @param tileY The Y coordinate of the tile to construct
     * @param bd Tile's borders' data to be maintained for this tile based on the info in the cache
     */
    bool getConstrainedBorderVerticesForTile(const int& tileX, const int& tileY, BordersData& bd);

    /**
     * Stores the borders to preserve for a given tile in the cache. Note that the eastern/western/northern/southern vectors already include the corners, so we don't set them separately as opposed to getConstrainedBorderVerticesForTile
     *
     * @param tileX The X coordinate of the tile
     * @param tileY The Y coordinate of the tile
     * @param bd Tile's borders' data (to be preserved in new tiles)
     * @return
     */
    bool setConstrainedBorderVerticesForTile( const int& tileX, const int& tileY, BordersData& bd ) ;

    /**
     * Get the number of cache entries
     *
     * @return Number of cache entries
     */
    int numCacheEntries() { return m_mapTileToBorderVertices.size(); }

    /**
     * @brief Checks if a tile is visited
     *
     * WARNING: Does not perform bounds check for the tile
     * @param tileInd Tile index (a pair)
     * @return boolean indicating whether the tile was already visited
     */
    bool isTileVisited( const int& tileX, const int& tileY ) const {
        return m_tilesVisited[tileX-m_zoomBounds.getMinX()][tileY-m_zoomBounds.getMinY()];
    }

    /**
     * @brief Checks if a tile is being processed
     *
     * WARNING: Does not perform bounds check for the tile
     * @param tileX X coordinate of the tile
     * @param tileY Y coordinate of the tile
     * @return boolean indicating whether the tile was already processed
     */
    bool isTileBeingProcessed( const int& tileX, const int& tileY ) const {
        return m_tilesBeingProcessed[tileX-m_zoomBounds.getMinX()][tileY-m_zoomBounds.getMinY()];
    }

    void setBeingProcessed( const int& tileX, const int& tileY, bool b ) {
        m_tilesBeingProcessed[tileX-m_zoomBounds.getMinX()][tileY-m_zoomBounds.getMinY()] = b;
    }

    void setVisited( const int& tileX, const int& tileY, bool b ) {
        m_tilesVisited[tileX-m_zoomBounds.getMinX()][tileY-m_zoomBounds.getMinY()] = b;
    }

    /**
     * @brief Checks if a tile can start processing
     * @param tileX X coordinate of the tile
     * @param tileY Y coordinate of the tile
     * @return boolean indicating whether a tile can start processing
     */
    bool canTileStartProcessing( const int& tileX, const int& tileY ) ;

    /**
     * @brief Checks if all the tiles in the zoom have been processed
     */
    bool allTilesProcessed() const { return m_numProcessedTiles >= m_numTiles ; }

    /**
     * @brief Gets the number of processed tiles
     */
    int getNumProcessed() const { return m_numProcessedTiles ; }

    /**
     * @brief Gets the total amount of tiles to be processed in the zoom
     */
    int getNumTiles() const { return m_numTiles ; }

    /**
     * Simple internal drawing function for debugging purposes. Shows the state of the cache
     * @param curX The currently built tile X coordinate
     * @param curY The currently built tile Y coordinate
     */
    void showStatus(int curX = -1, int curY = -1, bool drawUnixTerminalColors = false) const;

private:
    // --- Attributes ---
    ctb::TileBounds m_zoomBounds;
    int m_tileMaxCoord;
    int m_numTiles;
    int m_numProcessedTiles;
    std::unordered_map<std::pair<int,int>, TileBorderVertices, boost::hash<std::pair<int, int>>> m_mapTileToBorderVertices;
    std::vector<std::vector<bool>> m_tilesVisited;
    std::vector<std::vector<bool>> m_tilesBeingProcessed;

    /// Bounds check for a tile (pair)
    bool isTileInBounds( const std::pair<int, int>& tileInd ) const {
        return isTileInBounds(tileInd.first, tileInd.second);
    }

    /// Bounds check for a tile (X/Y indices)
    bool isTileInBounds( const int& tileX, const int& tileY ) const {
        return (tileX >= m_zoomBounds.getMinX() && tileX <= m_zoomBounds.getMaxX() &&
                tileY >= m_zoomBounds.getMinY() && tileY <= m_zoomBounds.getMaxY());
    }
};

#endif //EMODNET_QMGC_ZOOM_TILES_BORDER_VERTICES_CACHE_H
