//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H
#define EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H

#include <ctb.hpp>

// Note: this set of classes implement a Strategy Pattern

// Strategy (algorithm interphase)
class ZoomTilesSchedulerBase
{
public:
    ZoomTilesSchedulerBase() : m_index(0), m_tilesToProcess() {}

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



// Concrete strategy
class ZoomTilesSchedulerRowwise : public ZoomTilesSchedulerBase
{
public:
    ZoomTilesSchedulerRowwise() : ZoomTilesSchedulerBase() {}

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



// Context: Allows to change the algorithm at runtime
class ZoomTilesScheduler
{
public:
    ZoomTilesScheduler( ZoomTilesSchedulerBase* scheduler ) { m_scheduler = scheduler ;}

    /// Allows to change the scheduler algorithm at runtime
    void setScheduler(ZoomTilesSchedulerBase* scheduler ) { m_scheduler = scheduler ;}

    // Replication of the ZoomTilesSchedulerBase interphase
    void initSchedule(const ctb::TileBounds& zoomBounds) { m_scheduler->initSchedule(zoomBounds) ; }
    ctb::TilePoint getNextTile() { return m_scheduler->getNextTile() ; }
    bool finished() { return m_scheduler->finished() ; }
    int numTiles() { return m_scheduler->numTiles() ; }
    int currentIndex() { return m_scheduler->currentIndex() ; }

private:
    ZoomTilesSchedulerBase* m_scheduler ;
};


// TODO: implement more schedulers!

#endif //EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H
