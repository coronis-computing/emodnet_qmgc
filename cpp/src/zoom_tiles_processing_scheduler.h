//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H
#define EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H

#include <ctb.hpp>

class ZoomTilesProcessingSchedulerBase
{
public:

// Pure virtual function defining the interphase for the possible processing orders to be implemented
virtual std::vector<ctb::TilePoint> tilesProcessingSchedule(const int& zoomLevel, const ctb::TileBounds& zoomBounds ) const = 0 ;

};

class ZoomTilesProcessingSchedulerRowwise : public ZoomTilesProcessingSchedulerBase
{
public:

// Pure virtual function defining the interphase for the possible processing orders to be implemented
virtual std::vector<ctb::TilePoint> tilesProcessingSchedule(const int& zoomLevel, const ctb::TileBounds& zoomBounds ) const
{
    std::vector<ctb::TilePoint> tilesToProcess ;

    for (int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++) {
        for (int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx++) {
            tilesToProcess.push_back(ctb::TilePoint(tx, ty));
        }
    }

    return tilesToProcess ;
}

};

// TODO: implement more schedulers!

#endif //EMODNET_TOOLS_ZOOM_TILES_PROCESSING_SCHEDULER_BASE_H
