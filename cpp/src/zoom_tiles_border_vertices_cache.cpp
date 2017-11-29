//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "zoom_tiles_border_vertices_cache.h"
#include <iostream>



bool ZoomTilesBorderVerticesCache::getConstrainedBorderVerticesForTile( const int& tileX, const int& tileY,
                                                                        std::vector<Point_3>& easternVerticesToPreserve,
                                                                        std::vector<Point_3>& westernVerticesToPreserve,
                                                                        std::vector<Point_3>& northernVerticesToPreserve,
                                                                        std::vector<Point_3>& southernVerticesToPreserve )
{
    easternVerticesToPreserve.clear() ;
    westernVerticesToPreserve.clear() ;
    northernVerticesToPreserve.clear() ;
    southernVerticesToPreserve.clear() ;

    /* Eastern vertices */
    // They come from tile tileX+1, tileY, if exists...
    int tileInd = linearInd( tileX+1, tileY ) ;
    if ( m_mapTileToBorderVertices.count(tileInd) ) {
        // ... and from the western border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[tileInd].getWesternVertices() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( it->coord, m_tileMaxCoord, it->height ) ;
            easternVerticesToPreserve.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[tileInd].isAlive() )
            m_mapTileToBorderVertices.erase(tileInd);
    }

    /* Western vertices */
    // They come from tile tileX-1, tileY, if exists...
    tileInd = linearInd( tileX-1, tileY ) ;
    if ( m_mapTileToBorderVertices.count(tileInd) ) {
        // ... and from the eastern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[tileInd].getEasternVertices() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( 0.0, it->coord, it->height ) ;
            westernVerticesToPreserve.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[tileInd].isAlive() )
            m_mapTileToBorderVertices.erase(tileInd) ;
    }

    /* Northern vertices */
    // They come from tile tileX, tileY+1, if exists...
    tileInd = linearInd( tileX, tileY+1 ) ;

    if ( m_mapTileToBorderVertices.count(tileInd) ) {
        // ... and from the southern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[tileInd].getSouthernVertices() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( it->coord, m_tileMaxCoord, it->height ) ;
            northernVerticesToPreserve.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[tileInd].isAlive() )
            m_mapTileToBorderVertices.erase(tileInd);
    }

    /* Southern vertices */
    // They come from tile tileX, tileY-1, if exists...
    tileInd = linearInd( tileX, tileY-1 ) ;

    if ( m_mapTileToBorderVertices.count(tileInd) ) {
        // ... and from the northern border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[tileInd].getNorthernVertices() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( it->coord, 0.0, it->height ) ;
            southernVerticesToPreserve.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[tileInd].isAlive() )
            m_mapTileToBorderVertices.erase(tileInd) ;
    }

    // Mark it as being processed
    tileInd = linearInd( tileX, tileY ) ;
    m_tilesBeingProcessed[tileInd] = true ;
}



bool ZoomTilesBorderVerticesCache::setConstrainedBorderVerticesForTile( const int& tileX, const int& tileY,
                                                                        const std::vector<Point_3>& easternVerticesToPreserve,
                                                                        const std::vector<Point_3>& westernVerticesToPreserve,
                                                                        const std::vector<Point_3>& northernVerticesToPreserve,
                                                                        const std::vector<Point_3>& southernVerticesToPreserve )
{
    std::vector<BorderVertex> easternBorderVertices, westernBorderVertices, northernBorderVertices, southernBorderVertices ;

    int numNeighsInBounds = 0 ;
    if ( tileX+1 <= m_zoomBounds.getMaxX() ) {
        int tileInd = linearInd( tileX+1, tileY ) ;
        if ( tileInd > 0 && tileInd < m_numTiles && !m_tilesVisited[tileInd] ) // The tile still needs to be built
        {
            numNeighsInBounds++;

            for (std::vector<Point_3>::const_iterator it = easternVerticesToPreserve.begin();
                 it != easternVerticesToPreserve.end(); ++it ) {
                easternBorderVertices.push_back( BorderVertex( it->y(), it->z() ) ) ;
            }
        }
    }
    if ( tileX-1 >= m_zoomBounds.getMinX() ) {
        int tileInd = linearInd( tileX-1, tileY ) ;
        if ( tileInd > 0 && tileInd < m_numTiles && !m_tilesVisited[tileInd] ) // The tile still needs to be built
        {
            numNeighsInBounds++;

            for (std::vector<Point_3>::const_iterator it = westernVerticesToPreserve.begin();
                 it != westernVerticesToPreserve.end(); ++it ) {
                westernBorderVertices.push_back( BorderVertex( it->y(), it->z() ) ) ;
            }
        }
    }
    if ( tileY+1 <= m_zoomBounds.getMaxY() ) {
        int tileInd = linearInd( tileX, tileY+1 ) ;
        if ( tileInd > 0 && tileInd < m_numTiles && !m_tilesVisited[tileInd] ) // The tile still needs to be built
        {
            numNeighsInBounds++;

            for (std::vector<Point_3>::const_iterator it = northernVerticesToPreserve.begin();
                 it != northernVerticesToPreserve.end(); ++it ) {
                northernBorderVertices.push_back( BorderVertex( it->x(), it->z() ) ) ;
            }
        }
    }
    if ( tileY-1 >= m_zoomBounds.getMinY() ) {
        int tileInd = linearInd( tileX, tileY-1 ) ;
        if ( tileInd > 0 && tileInd < m_numTiles && !m_tilesVisited[tileInd] ) // The tile still needs to be built
        {
            numNeighsInBounds++;

            for (std::vector<Point_3>::const_iterator it = southernVerticesToPreserve.begin();
                 it != southernVerticesToPreserve.end(); ++it ) {
                southernBorderVertices.push_back( BorderVertex( it->x(), it->z() ) ) ;
            }
        }
    }

    // Add the cache entry
    int tileInd = linearInd(tileX, tileY) ;
    if ( numNeighsInBounds > 0 ) {
        TileBorderVertices tbv(easternBorderVertices,
                               westernBorderVertices,
                               northernBorderVertices,
                               southernBorderVertices,
                               numNeighsInBounds);
        m_mapTileToBorderVertices[tileInd] = tbv ;
    }

    // Mark as visited
    m_tilesVisited[tileInd] = true ;

    // Update the number of processed tiles
    m_numProcessedTiles++ ;

    // Remove from the list of tiles being processed
    m_tilesBeingProcessed[tileInd] = false ;
}



bool ZoomTilesBorderVerticesCache::canTileStartProcessing( const int& tileX, const int& tileY ) const
{
    // Check that the 4 neighbors are either visited or not being processed
    int tileInd = linearInd(tileX-1, tileY) ;
    if (tileInd < 0 || isTileVisited(tileInd) || !isTileBeingProcessed(tileInd)) {
        tileInd = linearInd(tileX+1, tileY) ;
        if (tileInd < 0 || isTileVisited(tileInd) || !isTileBeingProcessed(tileInd)) {
            tileInd = linearInd(tileX, tileY-1) ;
            if (tileInd < 0 || isTileVisited(tileInd) || !isTileBeingProcessed(tileInd)) {
                tileInd = linearInd(tileX, tileY+1) ;
                if (tileInd < 0 || isTileVisited(tileInd) || !isTileBeingProcessed(tileInd))
                    return true ;
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false ;
    }
    else
        return false;
}