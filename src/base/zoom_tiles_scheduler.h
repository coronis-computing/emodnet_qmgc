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

#ifndef EMODNET_QMGC_ZOOM_SCHEDULER_H
#define EMODNET_QMGC_ZOOM_SCHEDULER_H

#include <ctb.hpp>

/**
 * @class ZoomTilesSchedulerStrategy
 *
 * @brief An instance of a ZoomTilesSchedulerStrategy rules the desired order at which tiles in a given zoom should be processed (see derived classes for concrete strategies).
 *
 * Note that the order in this schedule will only be the actual order in which the tiles will be processed when using a single processing thread.
 *
 * When processing in parallel (numThreads > 1), the schedule will only be a "preferred order", since the tiles that can be processed at a given moment depend on the tiles being processed in the other threads.
 * That is, when a tile is being processed, it prevents all its neighbors in an 8-connected vicinity from start processing. In those cases, the builder will select the closest one in the list of the scheduler that can be executed. Those which could not be executed are put on a waiting list, and they are processed as soon as possible.
 *
 * While a complete analysis of the implications of the parameters has not been performed, keep in mind that the number of threads used and the preferred order selected will have consequences on both the number of tiles that need to be stored in the cache to maintain already-built borders and the number of processes that can be spawned at a given moment during the execution of the pyramid builder.
 *
 * Note: this class is the algorithm interphase of an Strategy pattern.
 */
class ZoomTilesSchedulerStrategy
{
public:
    ZoomTilesSchedulerStrategy() : m_index(0), m_tilesToProcess() {}

    virtual void initSchedule(const ctb::TileBounds& zoomBounds) = 0 ;

    // The root is only composed of two tiles... and should be always present regardless of the computed zoom bounds
    void initRootSchedule() {
        m_index = 0;
        m_tilesToProcess.clear();
        m_tilesToProcess.emplace_back(ctb::TilePoint(0,0));
        m_tilesToProcess.emplace_back(ctb::TilePoint(1,0));
    }

    /// Get the next tile to process
    ctb::TilePoint getNextTile() {
        if ( finished() )
            return ctb::TilePoint(0,0) ; // dummy, should never happen
        ctb::TilePoint tp = m_tilesToProcess[m_index] ;
        m_index++ ;
        return tp ;
    }

    /// Number of tiles in the schedule
    int numTiles() { return m_tilesToProcess.size() ; }

    /// The current index in the schedule
    int currentIndex() { return m_index ; }

    /// Marks if the current schedule is finished
    bool finished() { return m_index >= m_tilesToProcess.size() ; }

protected:
    int m_index ;
    std::vector<ctb::TilePoint> m_tilesToProcess ;
};



/**
 * @class ZoomTilesScheduler
 * @brief Context class: Allows to change the algorithm at runtime
 */
class ZoomTilesScheduler
{
public:
    /// Default constructor (remember to set the strategy with setScheduler!)
    ZoomTilesScheduler(){}

    /// Constructor from an scheduler strategy
    ZoomTilesScheduler( std::shared_ptr<ZoomTilesSchedulerStrategy> scheduler ) { m_scheduler = scheduler ;}

    /// Allows to change the scheduler algorithm at runtime
    void setScheduler(std::shared_ptr<ZoomTilesSchedulerStrategy> scheduler ) { m_scheduler = scheduler ;}

    /// Replication of the ZoomTilesSchedulerBase interphase
    void initSchedule(const ctb::TileBounds& zoomBounds) { m_scheduler->initSchedule(zoomBounds) ; }

    void initRootSchedule() { m_scheduler->initRootSchedule(); }

    ctb::TilePoint getNextTile() { return m_scheduler->getNextTile(); }
    bool finished() { return m_scheduler->finished(); }
    int numTiles() { return m_scheduler->numTiles(); }
    int currentIndex() { return m_scheduler->currentIndex(); }

private:
    std::shared_ptr<ZoomTilesSchedulerStrategy> m_scheduler ;
};

// --- Concrete strategies ---

/**
 * @class ZoomTilesSchedulerRowwiseStrategy
 * @brief Row-wise scheduler
 *
 * Returns the indices of the tiles to be processed following a row-wise order.
 */
class ZoomTilesSchedulerRowwiseStrategy : public ZoomTilesSchedulerStrategy
{
public:
    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
        m_index = 0 ;
        m_tilesToProcess.clear() ;
        for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
            for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx++) {
                m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
            }
        }
    }
};



/**
 * @class ZoomTilesSchedulerColumnwiseStrategy
 * @brief Column-wise scheduler
 *
 * Returns the indices of the tiles to be processed following a column-wise order.
 */
class ZoomTilesSchedulerColumnwiseStrategy : public ZoomTilesSchedulerStrategy
{
public:
    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
        m_index = 0 ;
        m_tilesToProcess.clear() ;
        for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx++) {
            for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
                m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
            }
        }
    }
};



/**
 * @class ZoomTilesSchedulerChessboardStrategy
 * @brief Chessboard scheduler
 *
 * Considering a linear ordering of tiles (row-wise), the odd tiles are processed before the even ones.
 * Note: This requires ALL the border vertices to be maintained in the cache until the even ones are processed,
 * which may result in large memory requirements for deep levels of the pyramid
 */
class ZoomTilesSchedulerChessboardStrategy : public ZoomTilesSchedulerStrategy
{
public:
    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
        m_index = 0 ;
        m_tilesToProcess.clear() ;
        for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
            if ( ty%2 == 0 ) {
                for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx+=2) {
                    m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
                }
            }
            else {
                for (int tx = zoomBounds.getMinX()+1; tx <= zoomBounds.getMaxX(); tx+=2) {
                    m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
                }
            }
        }

        for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
            if ( ty%2 == 0 ) {
                for (int tx = zoomBounds.getMinX()+1; tx <= zoomBounds.getMaxX(); tx+=2) {
                    m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
                }
            }
            else {
                for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx+=2) {
                    m_tilesToProcess.push_back(ctb::TilePoint(tx, ty));
                }
            }
        }
    }
};


/**
 * @class ZoomTilesSchedulerRecursiveFourConnectedStrategy
 * @brief Recursive scheduler
 *
 * Starting with the center tile, it recursively adds tiles using a 4-connected neighborhood.
 */
class ZoomTilesSchedulerRecursiveFourConnectedStrategy : public ZoomTilesSchedulerStrategy
{
public:
    typedef std::vector<std::vector<bool>> VisitedMatrix ;

    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
        m_index = 0 ;
        m_tilesToProcess.clear() ;

        unsigned int rows = zoomBounds.getMaxY()-zoomBounds.getMinY()+1 ;
        unsigned int cols = zoomBounds.getMaxX()-zoomBounds.getMinX()+1 ;

        VisitedMatrix visited;
        visited.resize(rows);

        for(int i=0; i < rows; i++)
        {
            visited[i].resize(cols) ;
            for(int j=0; j < cols; j++)
                visited[i][j] = false ;
        }

        unsigned int midY = zoomBounds.getMinY() + ( (zoomBounds.getMaxY()-zoomBounds.getMinY())/2 ) ;
        unsigned int midX = zoomBounds.getMinX() + ( (zoomBounds.getMaxX()-zoomBounds.getMinX())/2 ) ;

        recursiveAddTiles( midX, midY, zoomBounds, visited ) ;
    }

private:
    void recursiveAddTiles(const int& tx, const int& ty, const ctb::TileBounds& zoomBounds, VisitedMatrix& visited ) {
        if ( inBounds( tx, ty, zoomBounds ) ) {
            m_tilesToProcess.push_back(ctb::TilePoint(tx, ty)) ;

            int txx = tx-zoomBounds.getMinX() ;
            int tyy = ty-zoomBounds.getMinY() ;

            visited[tyy][txx] = true ;
            // Recurse up
            if ( inBounds(tx+1, ty, zoomBounds) && !visited[tyy][txx+1] )
                recursiveAddTiles(tx+1, ty, zoomBounds, visited ) ;
            // Recurse down
            if ( inBounds(tx-1, ty, zoomBounds) && !visited[tyy][txx-1] )
                recursiveAddTiles(tx-1, ty, zoomBounds, visited ) ;
            // Recurse left
            if ( inBounds(tx, ty-1, zoomBounds) && !visited[tyy-1][txx] )
                recursiveAddTiles(tx, ty-1, zoomBounds, visited ) ;
            // Recurse right
            if ( inBounds(tx, ty+1, zoomBounds) && !visited[tyy+1][txx] )
                recursiveAddTiles(tx, ty+1, zoomBounds, visited ) ;
        }
    }

    bool inBounds( const int& tx, const int& ty, const ctb::TileBounds& zoomBounds ) const {
        return tx <= zoomBounds.getMaxX() && tx >= zoomBounds.getMinX() &&
               ty <= zoomBounds.getMaxY() && ty >= zoomBounds.getMinY() ;
    }
};

#include <queue>


/**
 * @class ZoomTilesSchedulerFourConnectedStrategy
 * @brief Recursive scheduler
 *
 * Starting with the center tile, it adds tiles using a 4-connected neighborhood. It uses a queue instead of a recursive call as in ZoomTilesSchedulerRecursiveFourConnectedStrategy.
 */
class ZoomTilesSchedulerFourConnectedStrategy : public ZoomTilesSchedulerStrategy
{
public:
    typedef std::vector<std::vector<bool>> VisitedMatrix ;

    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
        m_index = 0 ;
        m_tilesToProcess.clear() ;

        // Initialize the "visited" matrix
        unsigned int rows = zoomBounds.getMaxY()-zoomBounds.getMinY()+1 ;
        unsigned int cols = zoomBounds.getMaxX()-zoomBounds.getMinX()+1 ;

        VisitedMatrix visited;
        visited.resize(rows);

        for(int i=0; i < rows; i++)
        {
            visited[i].resize(cols) ;
            for(int j=0; j < cols; j++)
                visited[i][j] = false ;
        }

        // Middle point
        unsigned int midY = zoomBounds.getMinY() + ( (zoomBounds.getMaxY()-zoomBounds.getMinY())/2 ) ;
        unsigned int midX = zoomBounds.getMinX() + ( (zoomBounds.getMaxX()-zoomBounds.getMinX())/2 ) ;
        ctb::TilePoint midPt( midX, midY ) ;

        // Init the queue with the middle point
        std::queue<ctb::TilePoint> queue ;
        queue.push(midPt) ;

        while (!queue.empty()) {
            // Pop the next value on the queue
            ctb::TilePoint tp = queue.front() ;
            queue.pop() ;

            int tx = tp.x ;
            int ty = tp.y ;
            int txx = tx-zoomBounds.getMinX() ;
            int tyy = ty-zoomBounds.getMinY() ;

            if (!visited[tyy][txx]) {
                // Add the tile if not visited
                m_tilesToProcess.push_back(tp) ;
                visited[tyy][txx] = true ;

                // Visit up
                if ( inBounds(tx+1, ty, zoomBounds) && !visited[tyy][txx+1] )
                    queue.push(ctb::TilePoint(tx+1, ty)) ;
                // Visit down
                if ( inBounds(tx-1, ty, zoomBounds) && !visited[tyy][txx-1] )
                    queue.push(ctb::TilePoint(tx-1, ty)) ;
                // Visit left
                if ( inBounds(tx, ty-1, zoomBounds) && !visited[tyy-1][txx] )
                    queue.push(ctb::TilePoint(tx, ty-1)) ;
                // Visit right
                if ( inBounds(tx, ty+1, zoomBounds) && !visited[tyy+1][txx] )
                    queue.push(ctb::TilePoint(tx, ty+1)) ;
            }
        }
    }

private:
    bool inBounds( const int& tx, const int& ty, const ctb::TileBounds& zoomBounds ) const {
        return tx <= zoomBounds.getMaxX() && tx >= zoomBounds.getMinX() &&
               ty <= zoomBounds.getMaxY() && ty >= zoomBounds.getMinY() ;
    }
};

#endif //EMODNET_QMGC_ZOOM_SCHEDULER_H
