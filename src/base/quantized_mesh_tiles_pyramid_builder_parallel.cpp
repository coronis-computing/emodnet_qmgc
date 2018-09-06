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
//        ctb::TileCoordinate ll = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getLowerLeft(), zoom);
//        ctb::TileCoordinate ur = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getUpperRight(), zoom);
//        ctb::TileBounds zoomBounds(ll, ur);
        ctb::TileBounds zoomBounds;
        if (zoom == 0) {
            zoomBounds = ctb::TileBounds(ctb::TileCoordinate(0,0,0), ctb::TileCoordinate(0,1,0));
        }
        else {
            ctb::TileCoordinate ll = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getLowerLeft(), zoom);
            ctb::TileCoordinate ur = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getUpperRight(), zoom);
            zoomBounds = ctb::TileBounds(ll, ur);
        }

        std::cout << "--- Zoom " << zoom << " (" << zoomBounds.getMinX() << ", " << zoomBounds.getMinY() << ") --> (" << zoomBounds.getMaxX() << ", " << zoomBounds.getMaxY() << ") ---" << std::endl ;

        // Prepare a new borders' cache
        m_bordersCache = ZoomTilesBorderVerticesCache(zoomBounds, m_tilers[0].getOptions().HeighMapSamplingSteps-1);

        // Get the preferred ordering of processing
        if (zoom == 0)
            m_scheduler.initRootSchedule(); // Special schedule for the root, forcing the two tiles to be built
        else
            m_scheduler.initSchedule( zoomBounds ) ;

        // Set the parameters of the tin creators for this zoom
        for (int t = 0; t < m_numThreads; t++)
            m_tilers[t].setTinCreatorParamsForZoom(zoom);

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
                BordersData bd;
//                std::vector<Point_3> tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices ;
                m_bordersCache.getConstrainedBorderVerticesForTile(tp.x, tp.y, bd);

//                std::cout << "bd.tileEastVertices.size() = " << bd.tileEastVertices.size() << std::endl;
//                std::cout << "bd.tileWestVertices.size() = " << bd.tileWestVertices.size() << std::endl;
//                std::cout << "bd.tileNorthVertices.size() = " << bd.tileNorthVertices.size() << std::endl;
//                std::cout << "bd.tileSouthVertices.size() = " << bd.tileSouthVertices.size() << std::endl;

                std::future<BordersData> f = std::async( std::launch::async,
                                                         &QuantizedMeshTilesPyramidBuilderParallel::createTile, this,
                                                         coord, numThread, outDir, bd ) ; // Note: Using std::ref(bd) does not work, as we use bd as the future return value...

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


void QuantizedMeshTilesPyramidBuilderParallel::createTmsPyramidUnconstrainedBorders(const int &startZoom,
                                                                                    const int &endZoom,
                                                                                    const std::string &outDir,
                                                                                    const std::string &debugDir)
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
        ctb::TileBounds zoomBounds;
        if (zoom == 0) {
            zoomBounds = ctb::TileBounds(ctb::TileCoordinate(0,0,0), ctb::TileCoordinate(0,1,0));
        }
        else {
            ctb::TileCoordinate ll = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getLowerLeft(), zoom);
            ctb::TileCoordinate ur = m_tilers[0].grid().crsToTile(m_tilers[0].bounds().getUpperRight(), zoom);
            zoomBounds = ctb::TileBounds(ll, ur);
        }
        std::cout << "--- Zoom " << zoom << " (" << zoomBounds.getMinX() << ", " << zoomBounds.getMinY() << ") --> (" << zoomBounds.getMaxX() << ", " << zoomBounds.getMaxY() << ") ---" << std::endl ;

        // Get the preferred ordering of processing
//        if (zoom == 0)
//            m_scheduler.initRootSchedule(); // Special schedule for the root, forcing the two tiles to be built
//        else
            m_scheduler.initSchedule( zoomBounds ) ;

        int numLaunchedProcesses = 0 ; // Number of launched child processes in total
        while (!m_scheduler.finished()) {
            // Throw as many threads as possible to run in parallel
            int numThread = 0; // Number of threads thrown in this iteration
            std::vector<std::future<BordersData> > futures ;
            while (numThread < m_numThreads && !m_scheduler.finished()) {
                ctb::TilePoint tp = m_scheduler.getNextTile();

                numLaunchedProcesses++ ;

                std::cout << "Processing tile " << numLaunchedProcesses << "/" << m_scheduler.numTiles()
                          << ": x = " << tp.x << ", y = " << tp.y
                          << " (thread " << numThread << ")"
                          << std::endl ;

                ctb::TileCoordinate coord(zoom, tp.x, tp.y);

                // Launch thread
                BordersData bd; // empty borders...
                std::future<BordersData> f = std::async( std::launch::async,
                                                         &QuantizedMeshTilesPyramidBuilderParallel::createTile, this,
                                                         coord, numThread, outDir, bd ) ; // Note: Using std::ref(bd) does not work, as we use bd as the future return value...

                futures.emplace_back(std::move(f)) ;

                numThread++ ;
            }

            // Join the threads with the main thread and wait for the results to be ready
            for(auto &f : futures) {
                f.wait() ;
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



BordersData
QuantizedMeshTilesPyramidBuilderParallel::createTile( const ctb::TileCoordinate& coord,
                                                      const int& numThread,
                                                      const std::string& outDir,
                                                      const BordersData& bd )
{
    // Note: Using std::ref(bd) does not work, as we use bd as the future return value... So we copy the borders data
    BordersData bdC(bd) ;
//    bdC.tileEastVertices = bd.tileEastVertices;
//    bdC.tileWestVertices = bd.tileWestVertices;
//    bdC.tileNorthVertices = bd.tileNorthVertices;
//    bdC.tileSouthVertices = bd.tileSouthVertices;
    QuantizedMeshTile terrainTile = m_tilers[numThread].createTile(coord, bdC) ;

    // Write the file to disk (should be thread safe, as every thread will write to a different file, but we don't risk and use a mutex)
    m_diskWriteMutex.lock();
    const std::string fileName = getTileFileAndCreateDirs(coord, outDir);
    terrainTile.writeFile(fileName);

    // [DEBUG] Export the geometry of the tile in OFF format
    if (m_debugMode) {
        const std::string fileNameDebug = getDebugTileFileAndCreateDirs(coord);
        terrainTile.exportToOFF(fileNameDebug);
    }
    m_diskWriteMutex.unlock();

    // Free memory
//    delete terrainTile;

    // Return borders data
    return bdC ;
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
//    std::lock_guard<std::mutex> lock(m_diskWriteMutex);

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