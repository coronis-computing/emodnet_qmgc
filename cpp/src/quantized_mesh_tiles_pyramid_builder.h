//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
#define EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H

#include <ctb.hpp>
#include "quantized_mesh_tiler.h"
#include "zoom_tiles_processing_scheduler.h"
#include <iostream>



/**
 * @class
 *
 * Contains the logic to create the pyramid of tiles for the quantized mesh format.
 * Since the quantized mesh format requires coherence between neighboring tiles, this class is the responsible
 * of maintaining this coherence and build the tiles by taking into account already built tiles.
 */
class QuantizedMeshTilesPyramidBuilder {
public:
    QuantizedMeshTilesPyramidBuilder( const std::string& inputFile,
                                      const ctb::TilerOptions &options,
                                      const QuantizedMeshTiler::QMTOptions &qmtOptions,
                                      const ZoomTilesProcessingSchedulerBase* scheduler,
                                      const int& numThreads ) ;

    ~QuantizedMeshTilesPyramidBuilder() ;

    /**
     * @brief Creates the tile pyramid in quantized-mesh format
     *
     * Due to the quantized-mesh format requiring the vertices on the edges to coincide between neighbors, the creation
     * of the tiles' for each zoom is not as simple as in the heightmap format, and it requires a more complex loop taking
     * into account vertices at borders of the neighbors of the current tile being processed
     */
    void createTmsPyramid(const int &startZoom, const int &endZoom, const std::string &outDir) ;

    /**
     * @brief Check that the tile folder (zoom/x) exists, and creates it otherwise.
     *
     * @return The path of the tile's file.
     */
    static std::string getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                 const std::string &mainOutDir ) ;



private:
    // --- Attributes ---
    ctb::Grid m_grid ;
    QuantizedMeshTiler m_qmTiler ;
    int m_numThreads ;
    QuantizedMeshTiler **m_tilers ;
    const ZoomTilesProcessingSchedulerBase* m_scheduler ;

    // --- Functions ---
    std::vector<ctb::TilePoint> tilePointsToProcess( const int& zoomLevel, const ctb::TileBounds& zoomBounds ) ;
};


#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILES_PYRAMID_BUILDER_H
