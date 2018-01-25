//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "quantized_mesh_tiles_pyramid_builder_parallel.h"
#include <ctb.hpp>
#include "zoom_tiles_border_vertices_cache.h"
#include <future>




//QuantizedMeshTilesPyramidBuilderParallel::QuantizedMeshTilesPyramidBuilderParallel( const std::string& inputFile,
//                                  const ctb::TilerOptions &gdalTilerOptions,
//                                  const QuantizedMeshTiler::QMTOptions &qmtOptions,
//                                  const ZoomTilesScheduler& scheduler,
//                                  const int& numThreads )
//        : m_scheduler(scheduler)
//        , m_numThreads(numThreads)
//        , m_tilesWaitingToProcess()
//        , m_bordersCache()
//{
////    int tileSize = 256 ; // TODO: Check if this is ok...
////    ctb::Grid m_grid = ctb::GlobalGeodetic(tileSize);
//
//    const unsigned int numMaxThreads = std::thread::hardware_concurrency();
//    if ( m_numThreads <= 0 )
//        m_numThreads = numMaxThreads ;
//
//    // Create a tiler with its own pointer to the dataset for each thread
////    m_tilers = new QuantizedMeshTiler *[m_numThreads] ;
////    for ( int i = 0; i < m_numThreads; i++ ) {
////        GDALDataset  *poDataset = (GDALDataset *) GDALOpen(inputFile.c_str(), GA_ReadOnly);
////        if (poDataset == NULL) {
////            std::cerr << "Error: could not open GDAL dataset" << std::endl;
////            return;
////        }
////
////        m_tilers[i] = new QuantizedMeshTiler(poDataset, m_grid, gdalTilerOptions, qmtOptions, ) ;
////    }
//}


//QuantizedMeshTilesPyramidBuilderParallel::QuantizedMeshTilesPyramidBuilderParallel(const QuantizedMeshTiler& qmTiler,
//                                                                                   const ZoomTilesScheduler& scheduler,
//                                                                                   const int& numThreads )
//        : m_scheduler(scheduler), m_numThreads(numThreads), m_tiler(qmTiler), m_debugMode(false), m_debugDir("")
//{
//    const unsigned int numMaxThreads = std::thread::hardware_concurrency();
//    if ( m_numThreads <= 0 )
//        m_numThreads = numMaxThreads ;
//
//    // Create a tiler with its own pointer to the dataset for each thread
////    m_tilers = new QuantizedMeshTiler *[m_numThreads] ;
////    for ( int i = 0; i < m_numThreads; i++ ) {
////        m_tilers[i] = new QuantizedMeshTiler(qmTiler) ;
////    }
//}


QuantizedMeshTilesPyramidBuilderParallel::
QuantizedMeshTilesPyramidBuilderParallel(const std::vector<QuantizedMeshTiler>& qmTilers,
                                         const ZoomTilesScheduler& scheduler)
    : m_scheduler(scheduler), m_numThreads(qmTilers.size()), m_tilers(qmTilers), m_debugMode(false), m_debugDir("")
{
    const unsigned int numMaxThreads = std::thread::hardware_concurrency();
    if ( m_numThreads <= 0 )
        m_numThreads = numMaxThreads ;

//    std::cout << "m_tilers.size() = " << m_tilers.size() << std::endl ;

    // Create a tiler with its own pointer to the dataset for each thread
//    m_tilers = new QuantizedMeshTiler *[m_numThreads] ;
//    for ( int i = 0; i < m_numThreads; i++ ) {
//        m_tilers[i] = new QuantizedMeshTiler(qmTiler) ;
//    }
}


//
//
//QuantizedMeshTilesPyramidBuilderParallel::~QuantizedMeshTilesPyramidBuilderParallel()
//{
////    for ( int i = 0; i < m_numThreads; i++ )
////    {
////        delete m_tilers[i] ;
////    }
//}



void QuantizedMeshTilesPyramidBuilderParallel::createTmsPyramid(const int &startZoom, const int &endZoom, const std::string &outDir, const std::string& debugDir )
{
    // Set debug mode if needed
    if (!debugDir.empty()) {
        m_debugMode = true;
        m_debugDir = debugDir;
    }

    // Set the desired zoom levels to process
    int startZ = (startZoom < 0) ? m_tilers[0].maxZoomLevel() : startZoom ;
    int endZ = (endZoom < 0) ? 0 : endZoom;

    // Process one zoom at a time, just parallelize the tile generation within a zoom
    for (int zoom = startZ; zoom >= endZ; --zoom) {
        ctb::TileCoordinate ll = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getLowerLeft(), zoom);
        ctb::TileCoordinate ur = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getUpperRight(), zoom);

        ctb::TileBounds zoomBounds(ll, ur);

        std::cout << "--- Zoom " << zoom << " ---" << std::endl ;
        std::cout << "MinX = " << zoomBounds.getMinX() << std::endl ;
        std::cout << "MinY = " << zoomBounds.getMinY() << std::endl ;
        std::cout << "MaxX = " << zoomBounds.getMaxX() << std::endl ;
        std::cout << "MaxY = " << zoomBounds.getMaxY() << std::endl ;

        // Prepare a new borders' cache
        m_bordersCache = ZoomTilesBorderVerticesCache(zoomBounds, 256);

        // Get the preferred ordering of processing
        m_scheduler.initSchedule( zoomBounds ) ;
        std::cout << "Num tiles to process = " << m_scheduler.numTiles() << std::endl ;

        int numLaunchedProcesses = 0 ; // Number of launched child processes in total
        while (!m_bordersCache.allTilesProcessed()) {
            // Throw as many threads as possible to run in parallel
            int numThread = 0; // Number of threads thrown in this iteration
            std::vector<std::vector<Point_3>> tileEastVerticesVec, tileWestVerticesVec, tileNorthVerticesVec, tileSouthVerticesVec ;
            std::vector<ctb::TileCoordinate> coords ;
            std::vector<std::future<BordersData> > futures ;
            ctb::TilePoint tp;
            while (numThread < m_numThreads && getNextTileToProcess(tp)) {
                numLaunchedProcesses++ ;

                std::cout << "Processing tile " << numLaunchedProcesses << "/" << m_scheduler.numTiles()
                          << ": x = " << tp.x << ", y = " << tp.y
                          << " (thread " << numThread << ")"
                          << "(num. cache entries = " << m_bordersCache.numCacheEntries() << ")"
                          << std::endl ;

                ctb::TileCoordinate coord(zoom, tp.x, tp.y);
                coords.push_back(coord) ;

                // Get constraints at borders from cache
                std::vector<Point_3> tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ;
                m_bordersCache.getConstrainedBorderVerticesForTile(tp.x, tp.y, tileEastVertices, tileWestVertices,
                                                                   tileNorthVertices, tileSouthVertices);

                std::future<BordersData> f = std::async( std::launch::async,
                                                         &QuantizedMeshTilesPyramidBuilderParallel::createTile, this,
                                                         coord, numThread, outDir ) ;

                futures.emplace_back(std::move(f)) ;

                numThread++ ;
            }

            // Join the threads with the main thread and wait for the results to be ready
            for(auto &f : futures) {
                f.wait() ;
            }

            // Update cache for the next iteration
            for ( int i = 0; i < numThread; i++ ) {
                BordersData bd = futures[i].get() ;
                m_bordersCache.setConstrainedBorderVerticesForTile( coords[i].x, coords[i].y,
                                                                    bd.tileEastVertices, bd.tileWestVertices,
                                                                    bd.tileNorthVertices, bd.tileSouthVertices ) ;
            }
        }

    }
}



bool QuantizedMeshTilesPyramidBuilderParallel::getNextTileToProcess(ctb::TilePoint& tileXY)
{
    // Prioritize the processing of those tiles waiting because of restrictions in neighboring tiles (allows to clear memory from the cache when not needed anymore)
    for ( std::vector<ctb::TilePoint>::iterator it = m_tilesWaitingToProcess.begin();
          it != m_tilesWaitingToProcess.end(); ++it ) {
        if ( m_bordersCache.canTileStartProcessing(it->x, it->y) ) {
            tileXY = *it ;
            m_tilesWaitingToProcess.erase(it);
            return true ;
        }
    }

    // Otherwise, get the next tile to process from the scheduler's list

    // Check that we are in bounds
    if ( m_scheduler.finished() )
        return false ;

    // Look for the first tile that can be processed
    bool found = false ;
    while ( !m_scheduler.finished() && !found ) {
        tileXY = m_scheduler.getNextTile() ;
        found = m_bordersCache.canTileStartProcessing(tileXY.x, tileXY.y) ;
        if (!found) // Put the ones that cannot be processed yet into the waiting list
            m_tilesWaitingToProcess.push_back(tileXY);
    }

    if ( found ) {
        // We found a tile that can be processed
        return true; // tileXY contains the following tile to process
    }
    else
        return false ; // tileXY contains the last tile because we got to the end
}



QuantizedMeshTilesPyramidBuilderParallel::BordersData
QuantizedMeshTilesPyramidBuilderParallel::createTile( const ctb::TileCoordinate& coord,
                                                      const int& numThread,
                                                      const std::string& outDir )
{
    BordersData bd ;
    QuantizedMeshTile terrainTile = m_tilers[numThread].createTile( coord, bd.tileEastVertices, bd.tileWestVertices,
                                                               bd.tileNorthVertices, bd.tileSouthVertices ) ;

    // Write the file to disk (should be thread safe, as every thread will write to a different file)
    const std::string fileName = getTileFileAndCreateDirs(coord, outDir);
    terrainTile.writeFile(fileName);

    // [DEBUG] Export the geometry of the tile in OFF format
    if (m_debugMode) {
        const std::string fileNameDebug = getDebugTileFileAndCreateDirs(coord);
        terrainTile.exportToOFF(fileNameDebug);
    }

    // Free memory
//    delete terrainTile;

    // Return borders data
    return bd ;
}



std::string QuantizedMeshTilesPyramidBuilderParallel::getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
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




std::string QuantizedMeshTilesPyramidBuilderParallel::getDebugTileFileAndCreateDirs( const ctb::TileCoordinate &coord )
{
    // Check/create the tile folder (zoom/x)
    fs::path mainOutDirPath(m_debugDir) ;
    fs::path tileFolder = mainOutDirPath / fs::path(std::to_string(coord.zoom)) / fs::path(std::to_string(coord.x)) ;
    if ( !fs::exists( tileFolder ) && !fs::create_directories( tileFolder ) ) {
        std::cerr << "[ERROR] Cannot create the tile folder" << tileFolder << std::endl ;
        return std::string() ;
    }

    fs::path fileNamePath = tileFolder / fs::path(std::to_string(coord.y) + ".off") ;

    return fileNamePath.string() ;
}