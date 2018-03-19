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
    std::pair<int,int> tileInd = std::make_pair( tileX+1, tileY ) ;
    if ( m_mapTileToBorderVertices.count(tileInd) ) {
        // ... and from the western border of the previously-computed tile
        std::vector<BorderVertex> borderCoords = m_mapTileToBorderVertices[tileInd].getWesternVertices() ;

        // Conversion to 3D points
        for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        {
            Point_3 p( m_tileMaxCoord, it->coord, it->height ) ;
            easternVerticesToPreserve.push_back(p) ;
        }

        // Free some memory from the cache if the tile's borders info is not needed anymore
        if( !m_mapTileToBorderVertices[tileInd].isAlive() )
            m_mapTileToBorderVertices.erase(tileInd);
    }

    /* Western vertices */
    // They come from tile tileX-1, tileY, if exists...
    tileInd = std::make_pair( tileX-1, tileY ) ;
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
    tileInd = std::make_pair( tileX, tileY+1 ) ;

//    std::cout << "tileX, tileY-1 = " << tileX << ", " << tileY-1 << std::endl;
//    std::cout << "tileInd (n) = " << tileInd << std::endl;
    if ( m_mapTileToBorderVertices.count(tileInd) ) {
//        std::cout << "Get northern vertices" << std::endl;

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
    tileInd = std::make_pair( tileX, tileY-1 ) ;
//    std::cout << "tileX, tileY+1 = " << tileX << ", " << tileY+1 << std::endl;
//    std::cout << "tileInd (s) = " << tileInd << std::endl;
    if ( m_mapTileToBorderVertices.count(tileInd) ) {
//        std::cout << "Get southern vertices" << std::endl;

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
    tileInd = std::make_pair( tileX, tileY ) ;
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
    }
    if ( isTileInBounds(tileX, tileY-1) && !isTileVisited(tileX, tileY-1) ) {
        numNeighsInBounds++;
        for (std::vector<Point_3>::const_iterator it = southernVerticesToPreserve.begin();
             it != southernVerticesToPreserve.end(); ++it ) {
            southernBorderVertices.push_back( BorderVertex( it->x(), it->z() ) ) ;
        }
    }

    // Add the cache entry (if not empty)
    std::pair<int,int> tileInd = std::make_pair( tileX, tileY ) ;
    if ( numNeighsInBounds > 0 ) {
        TileBorderVertices tbv(easternBorderVertices,
                               westernBorderVertices,
                               northernBorderVertices,
                               southernBorderVertices,
                               numNeighsInBounds);
//
//        std::cout << "easternBorderVertices.size() = " << easternBorderVertices.size() << std::endl;
//        std::cout << "westernBorderVertices.size() = " << westernBorderVertices.size() << std::endl;
//        std::cout << "northernBorderVertices.size() = " << northernBorderVertices.size() << std::endl;
//        std::cout << "southernBorderVertices.size() = " << southernBorderVertices.size() << std::endl;

        m_mapTileToBorderVertices[tileInd] = tbv ;
    }

    // Mark the tile as visited
    m_tilesVisited[tileInd] = true ;

    // Update the number of processed tiles
    m_numProcessedTiles++ ;

    // Remove from the list of tiles being processed
    m_tilesBeingProcessed[tileInd] = false ;
}



//bool ZoomTilesBorderVerticesCache::canTileStartProcessing( const int& tileX, const int& tileY )
//{
//    // Check that the 4 neighbors are either visited or not being processed
//    std::pair<int, int> westTileInd = std::make_pair(tileX-1, tileY) ;
//    if (!isTileInBounds(westTileInd) || isTileVisited(westTileInd) || !isTileBeingProcessed(westTileInd)) {
//        std::pair<int, int> eastTileInd = std::make_pair(tileX+1, tileY) ;
//        if (!isTileInBounds(eastTileInd) || isTileVisited(eastTileInd) || !isTileBeingProcessed(eastTileInd)) {
//            std::pair<int, int> northTileInd = std::make_pair(tileX, tileY+1) ;
//            if (!isTileInBounds(northTileInd) || isTileVisited(northTileInd) || !isTileBeingProcessed(northTileInd)) {
//                std::pair<int, int> southTileInd = std::make_pair(tileX, tileY-1) ;
//                if (!isTileInBounds(southTileInd) || isTileVisited(southTileInd) || !isTileBeingProcessed(southTileInd))
//                    return true ;
//                else
//                    return false;
//            }
//            else
//                return false;
//        }
//        else
//            return false ;
//    }
//    else
//        return false;
//}

bool ZoomTilesBorderVerticesCache::canTileStartProcessing( const int& tileX, const int& tileY )
{
    // Check that the 8! neighbors are either visited or not being processed
    std::vector<std::pair<int,int>> eightConnNeigh ;
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY-1));

    for ( std::vector<std::pair<int,int>>::iterator it = eightConnNeigh.begin(); it != eightConnNeigh.end(); ++it ) {
//        if (!(!isTileInBounds(*it) || isTileVisited(*it) || !isTileBeingProcessed(*it)))
        if (isTileInBounds(*it) && !isTileVisited(*it) && isTileBeingProcessed(*it))
            return false;
    }
    return true ;
}