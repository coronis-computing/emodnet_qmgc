//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "quantized_mesh_tiles_pyramid_builder.h"
#include <ctb.hpp>
#include "zoom_tiles_border_vertices_cache.h"
#ifdef USE_OPENMP
#include <omp.h>
#endif



QuantizedMeshTilesPyramidBuilder::QuantizedMeshTilesPyramidBuilder( const std::string& inputFile,
                                  const ctb::TilerOptions &gdalTilerOptions,
                                  const QuantizedMeshTiler::QMTOptions &qmtOptions,
                                  const ZoomTilesProcessingSchedulerBase* scheduler,
                                  const int& numThreads )
        : m_scheduler(scheduler)
        , m_numThreads(numThreads)
{
    int tileSize = 256 ; // TODO: Check if this is ok...
    ctb::Grid m_grid = ctb::GlobalGeodetic(tileSize);

#ifdef USE_OPENMP
    const unsigned int numMaxThreads = omp_get_max_threads() ;
    if ( m_numThreads <= 0 )
        m_numThreads = numMaxThreads ;
    omp_set_num_threads( m_numThreads ) ;

    // Create a tiler with its own pointer to the dataset for each thread
    m_tilers = new QuantizedMeshTiler *[m_numThreads] ;
    for ( int i = 0; i < m_numThreads; i++ ) {
        GDALDataset  *poDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
        if (poDataset == NULL) {
            std::cerr << "Error: could not open GDAL dataset" << std::endl;
            return;
        }

        m_tilers[i] = new QuantizedMeshTiler(poDataset, m_grid, gdalTilerOptions, qmtOptions) ;
    }
#else
    m_numThreads = 1 ;
    m_tilers = new QuantizedMeshTiler *[1] ;
    GDALDataset  *poDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
    if (poDataset == NULL) {
        std::cerr << "Error: could not open GDAL dataset" << std::endl;
        return;
    }
    m_tilers[0] = new QuantizedMeshTiler(poDataset, m_grid, gdalTilerOptions, qmtOptions) ;
#endif
}



QuantizedMeshTilesPyramidBuilder::~QuantizedMeshTilesPyramidBuilder()
{
    delete m_scheduler ;
    for ( int i = 0; i < m_numThreads; i++ )
    {
        delete m_tilers[i] ;
    }
}



void QuantizedMeshTilesPyramidBuilder::createTmsPyramid(const int &startZoom, const int &endZoom, const std::string &outDir)
{
    // Set the desired zoom levels to process
    int startZ = (startZoom < 0) ? m_tilers[0]->maxZoomLevel() : startZoom ;
    int endZ = (endZoom < 0) ? 0 : endZoom;

    // Process one zoom at a time, just parallelize the tile generation within a zoom
    for (int zoom = startZ; zoom >= endZ; --zoom) {
        ctb::TileCoordinate ll = m_tilers[0]->grid().crsToTile( m_tilers[0]->bounds().getLowerLeft(), zoom ) ;
        ctb::TileCoordinate ur = m_tilers[0]->grid().crsToTile( m_tilers[0]->bounds().getUpperRight(), zoom ) ;

        ctb::TileBounds zoomBounds(ll, ur);

        // Prepare the borders cache
        ZoomTilesBorderVerticesCache bordersCache( zoomBounds, 256 ) ;

        // Process the tiles within the current zooom
        std::vector<ctb::TilePoint> tptp = m_scheduler->tilesProcessingSchedule( zoom, zoomBounds ) ;

#ifdef USE_OPENMP
        #pragma omp parallel for ordered schedule(dynamic)
#endif
        for (int i = 0; i < tptp.size(); i++ ) {
            ctb::TileCoordinate coord( zoom, tptp[i].x, tptp[i].y ) ;
            std::vector<Point_3> tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ;

#ifdef USE_OPENMP
        #pragma omp critical(getConstrainedBorderVerticesForTile)
            {
#endif
                std::cout << "Processing tile " << i+1 << "/" << tptp.size() << " in zoom = " << zoom << ": x = " << tptp[i].x << ", y = " << tptp[i].y << std::endl ;

                // Get the borders to maintain from the cache
                bordersCache.getConstrainedBorderVerticesForTile( tptp[i].x, tptp[i].y, tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ) ;
#ifdef USE_OPENMP
            }
#endif

#ifdef USE_OPENMP
            int threadId = omp_get_thread_num() ;
#else
            int threadId = 0 ;
#endif
            // Create the tile
            QuantizedMeshTile *terrainTile = m_tilers[threadId]->createTile(coord, tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ) ;

            // Write the file to disk
            const std::string fileName = getTileFileAndCreateDirs(coord, outDir);
            terrainTile->writeFile(fileName) ;

            // Free memory
            delete terrainTile ;

            // Update the cache
#ifdef USE_OPENMP
        #pragma omp critical(setConstrainedBorderVerticesForTile)
            {
#endif
            bordersCache.setConstrainedBorderVerticesForTile( tptp[i].x, tptp[i].y, tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ) ;
//                std::cout << "bordersCache.size() = " << bordersCache.size() << std::endl ;
#ifdef USE_OPENMP
            }
#endif
        }
    }
}



std::string QuantizedMeshTilesPyramidBuilder::getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                          const std::string &mainOutDir )
{
    // Check/create the tile folder (zoom/x)
    fs::path mainOutDirPath(mainOutDir) ;
    fs::path tileFolder = mainOutDirPath / fs::path(std::to_string(coord.zoom)) / fs::path(std::to_string(coord.x)) ;
    if ( !fs::exists( tileFolder ) && !fs::create_directories( tileFolder ) ) {
        std::cerr << "[ERROR] Cannot create the tile folder" << tileFolder << std::endl ;
        return std::string() ;
    }

    fs::path fileNamePath = tileFolder / fs::path(std::to_string(coord.y) + ".terrain") ;

    return fileNamePath.string() ;
}



std::vector<ctb::TilePoint> QuantizedMeshTilesPyramidBuilder::tilePointsToProcess( const int& zoomLevel, const ctb::TileBounds& zoomBounds )
{
    std::vector<ctb::TilePoint> tilesToProcess ;

//    if ( zoomLevel == 0 ) {
//        // Force the creation of the two base layers at level 0!
//        tilesToProcess.push_back(ctb::TilePoint(0, 0));
//        tilesToProcess.push_back(ctb::TilePoint(0, 1));
//    }
//    else {
        for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
            for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx++) {
                tilesToProcess.push_back(ctb::TilePoint(tx, ty));
            }
        }
//    }

    return tilesToProcess ;
}