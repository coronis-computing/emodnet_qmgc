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

#include "zoom_tiles_border_vertices_cache.h"
#include <iostream>



bool ZoomTilesBorderVerticesCache::getConstrainedBorderVerticesForTile(const int& tileX, const int& tileY, BordersData &bd)
{
    /* Eastern vertices */
    // They come from tile tileX+1, tileY, if exists...
    std::pair<int,int> easternTileInd = std::make_pair( tileX+1, tileY ) ;
    bool easternTileExists = m_mapTileToBorderVertices.count(easternTileInd);
    if ( easternTileExists ) {
        // ... and from the western border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[easternTileInd].getWesternVerticesAndDecreaseLife() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( m_tileMaxCoord, it->coord, it->height ) ;
            bd.tileEastVertices.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[easternTileInd].isAlive() )
            m_mapTileToBorderVertices.erase(easternTileInd);
    }

    /* Western vertices */
    // They come from tile tileX-1, tileY, if exists...
    std::pair<int,int> westernTileInd = std::make_pair( tileX-1, tileY ) ;
    bool westernTileExists = m_mapTileToBorderVertices.count(westernTileInd);
    if ( westernTileExists ) {
        // ... and from the eastern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[westernTileInd].getEasternVerticesAndDecreaseLife() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( 0.0, it->coord, it->height ) ;
            bd.tileWestVertices.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[westernTileInd].isAlive() )
            m_mapTileToBorderVertices.erase(westernTileInd) ;
    }

    /* Northern vertices */
    // They come from tile tileX, tileY+1, if exists...
    std::pair<int, int> northernTileInd = std::make_pair( tileX, tileY+1 ) ;
    bool northernTileExists = m_mapTileToBorderVertices.count(northernTileInd);
    if ( northernTileExists ) {
        // ... and from the southern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[northernTileInd].getSouthernVerticesAndDecreaseLife() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( it->coord, m_tileMaxCoord, it->height ) ;
            bd.tileNorthVertices.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[northernTileInd].isAlive() )
            m_mapTileToBorderVertices.erase(northernTileInd);
    }

    /* Southern vertices */
    // They come from tile tileX, tileY-1, if exists...
    std::pair<int,int> southernTileInd = std::make_pair( tileX, tileY-1 ) ;
    bool southernTileExists = m_mapTileToBorderVertices.count(southernTileInd);
    if ( southernTileExists ) {
        // ... and from the northern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[southernTileInd].getNorthernVerticesAndDecreaseLife() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( it->coord, 0.0, it->height ) ;
            bd.tileSouthVertices.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[southernTileInd].isAlive() )
            m_mapTileToBorderVertices.erase(southernTileInd);
    }

    // Special cases for the corners of the tile: when the 4-connected neighbors haven't been processed BUT some of the
    // diagonal neighbors (the rest of neighbors to get to 8-connectivity) have been processed.
    // In this case, we need to preserve the corresponding corner that overlaps with this tile!
    if (!northernTileExists && !westernTileExists) {
        // Check if north-west tile exists
        std::pair<int, int> northWestTileInd = std::make_pair(tileX-1, tileY+1);
        if (m_mapTileToBorderVertices.count(northWestTileInd)) {
            // Take the corner, in this case we look for the (0, m_tileMaxCoord) corner of the north-west tile
//            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[northWestTileInd].getSouthernVerticesAndDecreaseLife();
            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[northWestTileInd].getSouthernVertices();
            std::sort(borderCoords.begin(), borderCoords.end());

            // Which is the (0.0, m_tileMaxCoord) corner of the current tile
            bd.northWestCorner = Point_3(0.0, m_tileMaxCoord, borderCoords.rbegin()->height);
            bd.constrainNorthWestCorner = true;
        }
    }
    if (!northernTileExists && !easternTileExists) {
        // Check if north-east tile exists
        std::pair<int, int> northEastTileInd = std::make_pair(tileX+1, tileY+1);
        if (m_mapTileToBorderVertices.count(northEastTileInd)) {
            // Take the corner, in this case we look for the (0, 0) corner of the north-east tile
//            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[northEastTileInd].getSouthernVerticesAndDecreaseLife();
            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[northEastTileInd].getSouthernVertices();
            std::sort(borderCoords.begin(), borderCoords.end());

            // Which is the (m_tileMaxCoord, m_tileMaxCoord) corner of the current tile
            bd.northEastCorner = Point_3(m_tileMaxCoord, m_tileMaxCoord, borderCoords.begin()->height);
            bd.constrainNorthEastCorner = true;
        }
    }
    if (!southernTileExists && !westernTileExists) {
        // Check if south-west tile exists
        std::pair<int, int> southWestTileInd = std::make_pair(tileX-1, tileY-1);
        if (m_mapTileToBorderVertices.count(southWestTileInd)) {
            // Take the corner, in this case we look for the (m_tileMaxCoord, m_tileMaxCoord) corner of the south-west tile
//            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[southWestTileInd].getNorthernVerticesAndDecreaseLife();
            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[southWestTileInd].getNorthernVertices();
            std::sort(borderCoords.begin(), borderCoords.end());

            // Which is the (0.0, 0.0) corner of the current tile
            bd.southWestCorner = Point_3(0.0, 0.0, borderCoords.rbegin()->height);
            bd.constrainSouthWestCorner = true;
        }
    }
    if (!southernTileExists && !easternTileExists) {
        // Check if south-east tile exists
        std::pair<int, int> southEastTileInd = std::make_pair(tileX+1, tileY-1);
        if (m_mapTileToBorderVertices.count(southEastTileInd)) {
            // Take the corner, in this case we look for the (0, m_tileMaxCoord) corner of the south-east tile
//            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[southEastTileInd].getNorthernVerticesAndDecreaseLife();
            std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[southEastTileInd].getNorthernVertices();
            std::sort(borderCoords.begin(), borderCoords.end());

            // Which is the (m_tileMaxCoord, 0.0) corner of the current tile
            bd.southEastCorner = Point_3(m_tileMaxCoord, 0.0, borderCoords.begin()->height);
            bd.constrainSouthEastCorner = true;
        }
    }

    // Mark it as being processed
    std::pair<int, int> tileInd = std::make_pair( tileX, tileY ) ;
    m_tilesBeingProcessed[tileInd] = true ;
}



bool ZoomTilesBorderVerticesCache::setConstrainedBorderVerticesForTile( const int& tileX, const int& tileY,
                                                                        const std::vector<Point_3>& easternVerticesToPreserve,
                                                                        const std::vector<Point_3>& westernVerticesToPreserve,
                                                                        const std::vector<Point_3>& northernVerticesToPreserve,
                                                                        const std::vector<Point_3>& southernVerticesToPreserve )
{
    std::vector<BorderVertex> easternBorderVertices, westernBorderVertices, northernBorderVertices, southernBorderVertices ;

    // For each neighbor, check if the tile is in bounds and not visited yet
    // If these conditions hold, this means that the neighboring tile has not been built yet, so we collect the border
    // constraints that correspond to this neighbor for later use. The borders stored here will be collected later in
    // getConstrainedBorderVerticesForTile(...) function
    int numNeighsInBounds = 0 ;
    if ( isTileInBounds(tileX+1, tileY) && !isTileVisited(tileX+1, tileY) ) {
        numNeighsInBounds++;
        for (std::vector<Point_3>::const_iterator it = easternVerticesToPreserve.begin();
             it != easternVerticesToPreserve.end(); ++it ) {
            easternBorderVertices.push_back( BorderVertex( it->y(), it->z() ) ) ;
        }
    }
    if ( isTileInBounds(tileX-1, tileY) && !isTileVisited(tileX-1, tileY) ) { // The tile still needs to be built
        numNeighsInBounds++;
        for (std::vector<Point_3>::const_iterator it = westernVerticesToPreserve.begin();
             it != westernVerticesToPreserve.end(); ++it ) {
            westernBorderVertices.push_back( BorderVertex( it->y(), it->z() ) ) ;
        }
    }
    if ( isTileInBounds(tileX, tileY+1) && !isTileVisited(tileX, tileY+1) ) {
        numNeighsInBounds++;
        for (std::vector<Point_3>::const_iterator it = northernVerticesToPreserve.begin();
             it != northernVerticesToPreserve.end(); ++it ) {
            northernBorderVertices.push_back( BorderVertex( it->x(), it->z() ) ) ;
        }

//        // Check for NE and NW tiles, if they still need to be constructed update the number of neighbors (defines the number of times the cache can be read before deletion)
//        if ( isTileInBounds(tileX+1, tileY+1) && !isTileVisited(tileX+1, tileY+1) )
//            numNeighsInBounds++;
//        if ( isTileInBounds(tileX-1, tileY+1) && !isTileVisited(tileX-1, tileY+1) )
//            numNeighsInBounds++;
    }
    if ( isTileInBounds(tileX, tileY-1) && !isTileVisited(tileX, tileY-1) ) {
        numNeighsInBounds++;
        for (std::vector<Point_3>::const_iterator it = southernVerticesToPreserve.begin();
             it != southernVerticesToPreserve.end(); ++it ) {
            southernBorderVertices.push_back( BorderVertex( it->x(), it->z() ) ) ;
        }

//        // Check for SE and SW tiles, if they still need to be constructed update the number of neighbors (defines the number of times the cache can be read before deletion)
//        if ( isTileInBounds(tileX+1, tileY-1) && !isTileVisited(tileX+1, tileY-1) )
//            numNeighsInBounds++;
//        if ( isTileInBounds(tileX-1, tileY-1) && !isTileVisited(tileX-1, tileY-1) )
//            numNeighsInBounds++;
    }

    // Add the cache entry (if not empty)
    std::pair<int,int> tileInd = std::make_pair( tileX, tileY ) ;
    if ( numNeighsInBounds > 0 ) {
        TileBorderVertices tbv(easternBorderVertices,
                               westernBorderVertices,
                               northernBorderVertices,
                               southernBorderVertices,
                               numNeighsInBounds);

        m_mapTileToBorderVertices[tileInd] = tbv ;
    }

    // Mark the tile as visited
    m_tilesVisited[tileInd] = true ;

    // Update the number of processed tiles
    m_numProcessedTiles++ ;

    // Remove from the list of tiles being processed
    m_tilesBeingProcessed[tileInd] = false ;
}



bool ZoomTilesBorderVerticesCache::canTileStartProcessing( const int& tileX, const int& tileY )
{
    // Check that the 8! neighbors are either visited or not being processed
    std::vector<std::pair<int,int>> eightConnNeigh;
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY+1));

    for ( std::vector<std::pair<int,int>>::iterator it = eightConnNeigh.begin(); it != eightConnNeigh.end(); ++it ) {
        if (!(!isTileInBounds(*it) || isTileVisited(*it) || !isTileBeingProcessed(*it)))
            return false;
    }
    return true;
}