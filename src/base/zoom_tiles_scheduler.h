//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ZOOM_SCHEDULER_H
#define EMODNET_TOOLS_ZOOM_SCHEDULER_H

#include <ctb.hpp>

// Note: this set of classes implement a Strategy Pattern

// Strategy (algorithm interphase)
class ZoomTilesSchedulerStrategy
{
public:
    ZoomTilesSchedulerStrategy() : m_index(0), m_tilesToProcess() {}

    virtual void initSchedule(const ctb::TileBounds& zoomBounds) = 0 ;

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



// Context: Allows to change the algorithm at runtime
class ZoomTilesScheduler
{
public:
    /// Default constructor (remember to set the strategy with setScheduler!)
    ZoomTilesScheduler(){}

    /// Constructor from an scheduler strategy
    ZoomTilesScheduler( std::shared_ptr<ZoomTilesSchedulerStrategy> scheduler ) { m_scheduler = scheduler ;}

    /// Allows to change the scheduler algorithm at runtime
    void setScheduler(std::shared_ptr<ZoomTilesSchedulerStrategy> scheduler ) { m_scheduler = scheduler ;}

    // Replication of the ZoomTilesSchedulerBase interphase
    void initSchedule(const ctb::TileBounds& zoomBounds) { m_scheduler->initSchedule(zoomBounds) ; }
    ctb::TilePoint getNextTile() { return m_scheduler->getNextTile() ; }
    bool finished() { return m_scheduler->finished() ; }
    int numTiles() { return m_scheduler->numTiles() ; }
    int currentIndex() { return m_scheduler->currentIndex() ; }

private:
    std::shared_ptr<ZoomTilesSchedulerStrategy> m_scheduler ;
};

// --- Concrete strategies ---

// Row-wise strategy
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



// Column-wise strategy
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
 * Chessboard strategy. Considering a linear ordering of tiles (row-wise), the odd tiles are processed before the even ones.
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


class ZoomTilesSchedulerRecursiveFourConnectedStrategy : public ZoomTilesSchedulerStrategy
{
public:
    typedef std::vector<std::vector<bool>> VisitedMatrix ;

    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
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


class ZoomTilesSchedulerFourConnectedStrategy : public ZoomTilesSchedulerStrategy
{
public:
    typedef std::vector<std::vector<bool>> VisitedMatrix ;

    /// Initialize the schedule given the zoom bounds
    void initSchedule(const ctb::TileBounds& zoomBounds)
    {
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

        std::cout << "rows*cols = " << rows*cols << std::endl ;

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

#endif //EMODNET_TOOLS_ZOOM_SCHEDULER_H
