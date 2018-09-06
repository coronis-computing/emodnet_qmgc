//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
#define EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H

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
 * @class
 *
 * Contains the logic to create the pyramid of tiles for the quantized mesh format.
 * Since the quantized mesh format requires coherence between neighboring tiles, this class is the responsible
 * of maintaining this coherence and build the tiles by taking into account already built tiles.
 */
class QuantizedMeshTilesPyramidBuilderParallel
{
    // --- Private typedefs ---
    typedef typename TinCreation::Point_3 Point_3;

public:



//    QuantizedMeshTilesPyramidBuilderParallel( const std::string& inputFile,
//                                      const ctb::TilerOptions& options,
//                                      const QuantizedMeshTiler::QMTOptions& qmtOptions, // Note: we pass the options of the tiler and not the tiler itself because the pyramid builder may create more than one instance of a tiler
//                                      const ZoomTilesScheduler& scheduler,
//                                      const int& numThreads ) ;

//    /// Constructor with a single tiler
//    QuantizedMeshTilesPyramidBuilderParallel(const QuantizedMeshTiler& qmTiler,
//                                             const ZoomTilesScheduler& scheduler,
//                                             const int& numThreads);

    QuantizedMeshTilesPyramidBuilderParallel(const std::vector<QuantizedMeshTiler>& qmTilers,
                                             const ZoomTilesScheduler& scheduler);

//
//    ~QuantizedMeshTilesPyramidBuilderParallel() ;

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
     * @brief Check that the tile folder (zoom/x) exists, and creates it otherwise.
     *
     * @return The path of the tile's file.
     */
    static std::string getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                 const std::string &mainOutDir ) ;

    BordersData createTile( const ctb::TileCoordinate& coord,
                            const int& numThread,
                            const std::string& outDir,
                            const BordersData& bd ) ;

    /**
     * Get the next tile to process in the current zoom
     * @param tileXY
     * @return
     */
    bool getNextTileToProcess(ctb::TilePoint& tileXY) ;

private:
    // --- Attributes ---
//    ctb::Grid m_grid ;
//    QuantizedMeshTiler m_qmTiler ;
    int m_numThreads ;
//    QuantizedMeshTiler **m_tilers ;
//    QuantizedMeshTiler m_tiler ;
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


#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
