//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H
#define EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H

#include <ctb.hpp>
#include <unordered_map>
#include "tile_border_vertices.h"
#include "tin_creation/tin_creation_cgal_types.h"



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
            , m_zoomBoundsStepsX( zoomBounds.getMaxX()-zoomBounds.getMinX()+1 )
            , m_tileMaxCoord( tileMaxCoord )
            , m_numProcessedTiles(0)
    {
        m_numTiles = (zoomBounds.getMaxY()-zoomBounds.getMinY()+1)*(zoomBounds.getMaxX()-zoomBounds.getMinX()+1) ;
        m_tilesVisited = std::vector<bool>( m_numTiles, false ) ;
        m_tilesBeingProcessed = std::vector<bool>( m_numTiles, false ) ;
    }

    /**
     * Default Constructor
     */
    ZoomTilesBorderVerticesCache()
            : m_mapTileToBorderVertices()
            , m_zoomBounds()
            , m_zoomBoundsStepsX(0)
            , m_tileMaxCoord(0)
            , m_numProcessedTiles(0)
            , m_numTiles(0)
            , m_tilesVisited() {}

    /**
     * Given a tile, it gets the border edges to preserve from already constructed tiles
     * It also cleans up unnecessary data
     *
     * @param tileX The X coordinate of the tile to construct
     * @param tileY The Y coordinate of the tile to construct
     * @param easternVerticesToPreserve Eastern vertices to preserve for the next tile to construct
     * @param westernVerticesToPreserve Western vertices to preserve for the next tile to construct
     * @param northernVerticesToPreserve Southern vertices to preserve for the next tile to construct
     * @param southernVerticesToPreserve Northern vertices to preserve for the next tile to construct
     */
    bool getConstrainedBorderVerticesForTile( const int& tileX, const int& tileY,
                                              std::vector<Point_3>& easternVerticesToPreserve,
                                              std::vector<Point_3>& westernVerticesToPreserve,
                                              std::vector<Point_3>& northernVerticesToPreserve,
                                              std::vector<Point_3>& southernVerticesToPreserve ) ;
    /**
     * Stores the borders to preserve for a given tile in the cache
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
    int numCacheEntries() {return m_mapTileToBorderVertices.size() ;}

    bool isTileVisited( const int& tileX, const int& tileY ) const {
        int tileInd = linearInd(tileX, tileY) ;
        return isTileVisited(tileInd) ;
    }

    bool isTileVisited( const int& tileInd ) const {
        return tileInd >= 0 && tileInd < m_numTiles && m_tilesVisited[tileInd] ;
    }

    bool isTileBeingProcessed( const int& tileX, const int& tileY ) const {
        int tileInd = linearInd(tileX, tileY) ;
        return isTileBeingProcessed(tileInd) ;
    }

    bool isTileBeingProcessed( const int& tileInd ) const {
        return tileInd >= 0 && tileInd < m_numTiles && m_tilesBeingProcessed[tileInd] ;
    }

    bool canTileStartProcessing( const int& tileX, const int& tileY ) const ;

    bool allTilesProcessed() const { return m_numProcessedTiles >= m_numTiles ; }

    int getNumProcessed() const { return m_numProcessedTiles ; }

    int getNumTiles() const { return m_numTiles ; }

private:
    // --- Attributes ---
    ctb::TileBounds m_zoomBounds ;
    int m_zoomBoundsStepsX ; // For sub2ind transform of tile coordinates within the zoom
    int m_tileMaxCoord ;
    int m_numTiles ;
    int m_numProcessedTiles ;
    std::unordered_map<unsigned int, TileBorderVertices> m_mapTileToBorderVertices ;
    std::vector<bool> m_tilesVisited ; // Linearly indexed tiles, to check if they were already visited
    std::vector<bool> m_tilesBeingProcessed ; // Linearly indexed tiles, to check if they are processing at the moment

    // --- Private Functions ---
    int linearInd( const int& tileX, const int& tileY ) const {
        int tileInd = (tileY-m_zoomBounds.getMinY())* m_zoomBoundsStepsX + (tileX-m_zoomBounds.getMinX()) ;
        if (tileInd < 0 || tileInd >= m_numTiles)
            return -1 ;
        else
            return tileInd ;
    }
};

#endif //EMODNET_TOOLS_ZOOM_TILES_BORDER_VERTICES_CACHE_H
