//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H
#define EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H

#include <ctb.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "tile_border_vertices.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include "borders_data.h"



class ZoomTilesBorderVerticesCache
{
    // --- Private typedefs ---
    typedef TinCreation::Point_3 Point_3;

public:
    /**
     * Constructor
     *
     * @param zoomBounds The bounds of the current zoom
     * @param tileMaxCoord The number of pixels in a tile
     */
    ZoomTilesBorderVerticesCache( const ctb::TileBounds& zoomBounds, const int& tileMaxCoord, const bool& useSteinerVertices = false )
            : m_mapTileToBorderVertices()
            , m_zoomBounds(zoomBounds)
//            , m_zoomBoundsStepsX( zoomBounds.getMaxX()-zoomBounds.getMinX()+1 )
//            , m_zoomBoundsStepsX( zoomBounds.getMaxX()-zoomBounds.getMinX() )
            , m_tileMaxCoord( tileMaxCoord )
            , m_numProcessedTiles(0)
            , m_tilesVisited()
            , m_tilesBeingProcessed()
    {
        m_numTiles = (zoomBounds.getMaxY()-zoomBounds.getMinY()+1)*(zoomBounds.getMaxX()-zoomBounds.getMinX()+1) ;
//        m_tilesVisited = std::vector<bool>( m_numTiles, false ) ;
//        m_tilesBeingProcessed = std::vector<bool>( m_numTiles, false ) ;
    }

    /**
     * Default Constructor
     */
    ZoomTilesBorderVerticesCache()
            : m_mapTileToBorderVertices()
            , m_zoomBounds()
//            , m_zoomBoundsStepsX(0)
            , m_tileMaxCoord(0)
            , m_numProcessedTiles(0)
            , m_numTiles(0)
            , m_tilesVisited() {}

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
     * @param easternVerticesToPreserve Eastern vertices to preserve for the tile
     * @param westernVerticesToPreserve Western vertices to preserve for the tile
     * @param northernVerticesToPreserve Southern vertices to preserve for the tile
     * @param southernVerticesToPreserve Northern vertices to preserve for the tile
     * @return
     */
    bool setConstrainedBorderVerticesForTile( const int& tileX, const int& tileY,
                                              const std::vector<Point_3>& easternVerticesToPreserve,
                                              const std::vector<Point_3>& westernVerticesToPreserve,
                                              const std::vector<Point_3>& northernVerticesToPreserve,
                                              const std::vector<Point_3>& southernVerticesToPreserve ) ;

    /**
     * Get the number of cache entries
     *
     * @return Number of cache entries
     */
    int numCacheEntries() { return m_mapTileToBorderVertices.size(); }

    bool isTileVisited( const int& tileX, const int& tileY ) {
        std::pair<int,int> tileInd = std::make_pair( tileX, tileY ) ;
        return isTileVisited(tileInd) ;
    }

    // WARNING: Does not perform bounds check for the tile
    bool isTileVisited( const std::pair<int, int>& tileInd ) {
        if (m_tilesVisited.count(tileInd) == 0)
            return false;
        else
            return m_tilesVisited[tileInd] ;
    }

    bool isTileBeingProcessed( const int& tileX, const int& tileY ) {
        std::pair<int,int> tileInd = std::make_pair( tileX, tileY ) ;
        return isTileBeingProcessed(tileInd) ;
    }

    // WARNING: Does not perform bounds check for the tile
    bool isTileBeingProcessed( const std::pair<int,int>& tileInd ) {
        if (m_tilesBeingProcessed.count(tileInd) == 0)
            return false;
        else
            return m_tilesBeingProcessed[tileInd] ;
    }

    bool canTileStartProcessing( const int& tileX, const int& tileY ) ;

    bool allTilesProcessed() const { return m_numProcessedTiles >= m_numTiles ; }

    int getNumProcessed() const { return m_numProcessedTiles ; }

    int getNumTiles() const { return m_numTiles ; }

private:
    // --- Attributes ---
    ctb::TileBounds m_zoomBounds ;
//    int m_zoomBoundsStepsX ; // For sub2ind transform of tile coordinates within the zoom
    int m_tileMaxCoord ;
    int m_numTiles ;
    int m_numProcessedTiles ;
    std::unordered_map<std::pair<int,int>, TileBorderVertices, boost::hash<std::pair<int, int>>> m_mapTileToBorderVertices ;
    std::unordered_map<std::pair<int,int>, bool, boost::hash<std::pair<int, int>>> m_tilesVisited;
    std::unordered_map<std::pair<int,int>, bool, boost::hash<std::pair<int, int>>> m_tilesBeingProcessed;
//    std::vector<bool> m_tilesVisited ; // Linearly indexed tiles, to check if they were already visited
//    std::vector<bool> m_tilesBeingProcessed ; // Linearly indexed tiles, to check if they are being processed at the moment

//    // --- Private Functions ---
//    int linearInd( const int& tileX, const int& tileY ) const {
////        std::cout << "-------------------------------------" << std::endl;
////        std::cout << "tileX = " << tileX << std::endl;
////        std::cout << "tileY = " << tileY << std::endl;
////        std::cout << "m_zoomBounds.getMinY() = " << m_zoomBounds.getMinY() << std::endl;
////        std::cout << "m_zoomBounds.getMinX() = " << m_zoomBounds.getMinX() << std::endl;
////        std::cout << "m_zoomBoundsStepsX = " << m_zoomBoundsStepsX << std::endl;
////        std::cout << "m_numTiles = " << m_numTiles << std::endl;
//
//        int localX = (tileX-m_zoomBounds.getMinX());
//        if (localX < 0)
//            return -1;
//        int localY = (tileY-m_zoomBounds.getMinY());
//        if (localY < 0)
//            return -1;
//
//        int tileInd = localY*m_zoomBoundsStepsX + localX ;
//
////        std::cout << "tileInd = " << tileInd << std::endl;
//        if (tileInd >= 0 && tileInd < m_numTiles) {
////            std::cout << "ACCEPTED" << std::endl;
//            return tileInd;
//        }
//        else
//            return -1 ;
//    }

    bool isTileInBounds( const std::pair<int, int>& tileInd ) const {
        return isTileInBounds(tileInd.first, tileInd.second);
    }

    bool isTileInBounds( const int& tileX, const int& tileY ) const {
        return (tileX >= m_zoomBounds.getMinX() && tileX <= m_zoomBounds.getMaxX() &&
                tileY >= m_zoomBounds.getMinY() && tileY <= m_zoomBounds.getMaxY());
    }

};

#endif //EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H
