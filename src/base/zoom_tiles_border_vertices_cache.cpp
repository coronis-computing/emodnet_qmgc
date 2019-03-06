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

#include "zoom_tiles_border_vertices_cache.h"
#include <iostream>



bool ZoomTilesBorderVerticesCache::getConstrainedBorderVerticesForTile(const int& tileX, const int& tileY, BordersData &bd)
{
    // Mark it as being processed
    setBeingProcessed(tileX, tileY, true);

    // Check if the tile has an entry in the map
    std::pair<int,int> tileInd = std::make_pair( tileX, tileY ) ;
    int tileBorderDataExists = m_mapTileToBorderVertices.count(tileInd);

    if (tileBorderDataExists == 0) {
        // Nothing to maintain, return an empty BordersData structure
        bd = BordersData();
        return false; // No border vertices to maintain
    }

    // Convert the data from the map to 3D points
    TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];

    // Eastern border
    std::vector<BorderVertex> borderCoords = tbv.getEasternVertices();
    for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        bd.tileEastVertices.push_back(Point_3(m_tileMaxCoord, it->coord, it->height));
    // Western border
    borderCoords = tbv.getWesternVertices();
    for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        bd.tileWestVertices.push_back(Point_3(0.0, it->coord, it->height));
    // Northern border
    borderCoords = tbv.getNorthernVertices();
    for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        bd.tileNorthVertices.push_back(Point_3(it->coord, m_tileMaxCoord, it->height));
    // Southern border
    borderCoords = tbv.getSouthernVertices();
    for ( std::vector<BorderVertex>::iterator it = borderCoords.begin(); it != borderCoords.end(); ++it )
        bd.tileSouthVertices.push_back(Point_3(it->coord, 0.0, it->height));

    // Corners
    if (tbv.hasNorthWestCorner()) {
        bd.northWestCorner = Point_3(0.0, m_tileMaxCoord, m_mapTileToBorderVertices[tileInd].getNorthWestCorner());
        bd.constrainNorthWestCorner = true;
    }
    if (tbv.hasNorthEastCorner()) {
        bd.northEastCorner = Point_3(m_tileMaxCoord, m_tileMaxCoord, m_mapTileToBorderVertices[tileInd].getNorthEastCorner());
        bd.constrainNorthEastCorner = true;
    }
    if (tbv.hasSouthWestCorner()) {
        bd.southWestCorner = Point_3(0.0, 0.0, m_mapTileToBorderVertices[tileInd].getSouthWestCorner());
        bd.constrainSouthWestCorner = true;
    }
    if (tbv.hasSouthEastCorner()) {
        bd.southEastCorner = Point_3(m_tileMaxCoord, 0.0, m_mapTileToBorderVertices[tileInd].getSouthEastCorner());
        bd.constrainSouthEastCorner = true;
    }

    // --- Debug (start) ---
//    std::cout << "Constrained border vertices for tile:" << std::endl;
//    std::cout << "- EastVertices = " << bd.tileEastVertices.size() << std::endl;
//    std::cout << "- WestVertices = " << bd.tileWestVertices.size() << std::endl;
//    std::cout << "- NorthVertices = " << bd.tileNorthVertices.size() << std::endl;
//    std::cout << "- SouthVertices = " << bd.tileSouthVertices.size() << std::endl;
//    std::cout << "- constrainNorthWestCorner = " << bd.constrainNorthWestCorner << std::endl;
//    std::cout << "- constrainNorthEastCorner = " << bd.constrainNorthEastCorner << std::endl;
//    std::cout << "- constrainSouthWestCorner = " << bd.constrainSouthWestCorner << std::endl;
//    std::cout << "- constrainSouthEastCorner = " << bd.constrainSouthEastCorner << std::endl;
//    std::cout << "m_mapTileToBorderVertices.size before = " << m_mapTileToBorderVertices.size() << std::endl;
    // --- Debug  (end)  ---

    // Delete the borders data entry for the current tile
    m_mapTileToBorderVertices.erase(tileInd);

    // --- Debug (start) ---
//    std::cout << "m_mapTileToBorderVertices.size after = " << m_mapTileToBorderVertices.size() << std::endl;
    // --- Debug  (end)  ---

    return true;
}



bool ZoomTilesBorderVerticesCache::setConstrainedBorderVerticesForTile(const int& tileX, const int& tileY,
                                                                       BordersData &bd)
{
    // For each neighbor, check if the tile is in bounds and not visited yet
    // If these conditions hold, this means that the neighboring tile has not been built yet, so we store the border
    // constraints that correspond to this neighbor for later use. The borders stored here will be collected later in
    // getConstrainedBorderVerticesForTile(...) function

    // --- Debug (start) ---
//    std::cout << "setConstrainedBorderVerticesForTile: m_mapTileToBorderVertices.size before = " << m_mapTileToBorderVertices.size() << std::endl;
    // --- Debug  (end)  ---

    // TODO: Check if it is possible that a tile being processed is accessed here... should not be possible, but who knows...

    // The vertices to preserve in BorderVertex format
    std::shared_ptr<std::vector<BorderVertex>> easternBorderVertices, westernBorderVertices, northernBorderVertices, southernBorderVertices;
    easternBorderVertices = std::make_shared<std::vector<BorderVertex>>();
    westernBorderVertices = std::make_shared<std::vector<BorderVertex>>();
    northernBorderVertices = std::make_shared<std::vector<BorderVertex>>();
    southernBorderVertices = std::make_shared<std::vector<BorderVertex>>();
    for (std::vector<Point_3>::const_iterator it = bd.tileEastVertices.begin();
         it != bd.tileEastVertices.end(); ++it ) {
        easternBorderVertices->push_back( BorderVertex( it->y(), it->z() ) ) ;
    }
    for (std::vector<Point_3>::const_iterator it = bd.tileWestVertices.begin();
         it != bd.tileWestVertices.end(); ++it ) {
        westernBorderVertices->push_back( BorderVertex( it->y(), it->z() ) ) ;
    }
    for (std::vector<Point_3>::const_iterator it = bd.tileNorthVertices.begin();
         it != bd.tileNorthVertices.end(); ++it ) {
        northernBorderVertices->push_back( BorderVertex( it->x(), it->z() ) ) ;
    }
    for (std::vector<Point_3>::const_iterator it = bd.tileSouthVertices.begin();
         it != bd.tileSouthVertices.end(); ++it ) {
        southernBorderVertices->push_back( BorderVertex( it->x(), it->z() ) ) ;
    }

    // Extract and exclude the corners
    std::sort(easternBorderVertices->begin(), easternBorderVertices->end());
    std::sort(westernBorderVertices->begin(), westernBorderVertices->end());
    std::sort(northernBorderVertices->begin(), northernBorderVertices->end());
    std::sort(southernBorderVertices->begin(), southernBorderVertices->end());

    std::shared_ptr<double> northWestCorner, northEastCorner, southWestCorner, southEastCorner;
    northWestCorner = std::make_shared<double>(northernBorderVertices->begin()->height);
    northEastCorner = std::make_shared<double>(northernBorderVertices->rbegin()->height);
    southWestCorner = std::make_shared<double>(southernBorderVertices->begin()->height);
    southEastCorner = std::make_shared<double>(southernBorderVertices->rbegin()->height);

    easternBorderVertices->erase(easternBorderVertices->begin());
    easternBorderVertices->pop_back();
    westernBorderVertices->erase(westernBorderVertices->begin());
    westernBorderVertices->pop_back();
    northernBorderVertices->erase(northernBorderVertices->begin());
    northernBorderVertices->pop_back();
    southernBorderVertices->erase(southernBorderVertices->begin());
    southernBorderVertices->pop_back();

    // Create/Update cache entries for the neighboring tiles

    // For the NSWE border vertices, visit the NSWE neighbors:

    // Northern neighbor: set its southern vertices to maintain as the northern vertices of the current tile
//    TileBorderVertices tbv;
    std::pair<int, int> tileInd = std::make_pair(tileX, tileY+1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthernVertices())
            tbv.setSouthernVertices(northernBorderVertices);
    }
    // Southern neighbor: set its northern vertices to maintain as the southern vertices of the current tile
    tileInd = std::make_pair(tileX, tileY-1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthernVertices())
            tbv.setNorthernVertices(southernBorderVertices);
    }
    // Western neighbor: set its eastern vertices to maintain as the western vertices of the current tile
    tileInd = std::make_pair(tileX-1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasEasternVertices())
            tbv.setEasternVertices(westernBorderVertices);
    }
    // Eastern neighbor: set its western vertices to maintain as the eastern vertices of the current tile
    tileInd = std::make_pair(tileX+1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasWesternVertices())
            tbv.setWesternVertices(easternBorderVertices);
    }

    // For the corners, visit the 8-neighbors (each corner may need to be transferred to up to 3 tiles!)
    // NW corner from the current tile...
    // ... is SE corner of tile tileX-1, tileY+1 ...
    tileInd = std::make_pair(tileX-1, tileY+1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthEastCorner()) tbv.setSouthEastCorner(northWestCorner);
    }
    // ... and is NE corner of tile tileX-1, tileY ...
    tileInd = std::make_pair(tileX-1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthEastCorner()) tbv.setNorthEastCorner(northWestCorner);
    }
    // ... and is SW corner of tile tileX, tileY+1
    tileInd = std::make_pair(tileX, tileY+1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthWestCorner()) tbv.setSouthWestCorner(northWestCorner);
    }

    // SW corner of the current tile...
    // ... is SE corner of tile tileX-1, tileY ...
    tileInd = std::make_pair(tileX-1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthEastCorner()) tbv.setSouthEastCorner(southWestCorner);
    }
    // ... and is NE corner of tile tileX-1, tileY-1 ...
    tileInd = std::make_pair(tileX-1, tileY-1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthEastCorner()) tbv.setNorthEastCorner(southWestCorner);
    }
    // ... and is NW corner of tile tileX, tileY-1
    tileInd = std::make_pair(tileX, tileY-1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthWestCorner()) tbv.setNorthWestCorner(southWestCorner);
    }

    // NE corner of the current tile ...
    // ... is SE corner of tile tileX, tileY+1 ...
    tileInd = std::make_pair(tileX, tileY+1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthEastCorner()) tbv.setSouthEastCorner(northEastCorner);
    }
    // ... and is SW corner of tile tileX+1, tileY+1 ...
    tileInd = std::make_pair(tileX+1, tileY+1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthWestCorner()) tbv.setSouthWestCorner(northEastCorner);
    }
    // ... and is NW corner of tile tileX+1, tileY
    tileInd = std::make_pair(tileX+1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthWestCorner()) tbv.setNorthWestCorner(northEastCorner);
    }

    // SE corner of the current tile ...
    // ... is SW corner of tile tileX+1, tileY ...
    tileInd = std::make_pair(tileX+1, tileY);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasSouthWestCorner()) tbv.setSouthWestCorner(southEastCorner);
    }
    // ... and is NW corner of tile tileX+1, tileY-1
    tileInd = std::make_pair(tileX+1, tileY-1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthWestCorner()) tbv.setNorthWestCorner(southEastCorner);
    }
    // ... and is NE corner of tile tileX, tileY-1
    tileInd = std::make_pair(tileX, tileY-1);
    if ( isTileInBounds(tileInd.first, tileInd.second) && !isTileVisited(tileInd.first, tileInd.second) ) {
        TileBorderVertices& tbv = m_mapTileToBorderVertices[tileInd];
        if (!tbv.hasNorthEastCorner()) tbv.setNorthEastCorner(southEastCorner);
    }

    // Mark the tile as visited
    setVisited(tileX, tileY, true);

    // Update the number of processed tiles
    m_numProcessedTiles++ ;

    // Remove from the list of tiles being processed
    setBeingProcessed(tileX, tileY, false);

    // --- Debug (start) ---
//    std::cout << "setConstrainedBorderVerticesForTile: m_mapTileToBorderVertices.size after = " << m_mapTileToBorderVertices.size() << std::endl;
    // --- Debug  (end)  ---

    return true;
}



bool ZoomTilesBorderVerticesCache::canTileStartProcessing( const int& tileX, const int& tileY )
{
    // Check that the 8! neighbors are either visited or not being processed
    std::vector<std::pair<int,int>> eightConnNeigh;
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY+1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX-1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX+1, tileY));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY-1));
    eightConnNeigh.emplace_back(std::pair<int,int>(tileX, tileY+1));

    for ( std::vector<std::pair<int,int>>::iterator it = eightConnNeigh.begin(); it != eightConnNeigh.end(); ++it ) {
        if (!(!isTileInBounds(it->first, it->second) || isTileVisited(it->first, it->second) || !isTileBeingProcessed(it->first, it->second)))
            return false;
    }
    return true;
}



void ZoomTilesBorderVerticesCache::showStatus(int curX, int curY, bool drawUnixTerminalColors) const {
    for (int j = m_zoomBounds.getMinY(); j < m_zoomBounds.getMaxY()+1; j++) {
        for (int i = m_zoomBounds.getMinX(); i < m_zoomBounds.getMaxX()+1; i++) {
            if (curX == i && curY == j)
                if (drawUnixTerminalColors)
                    std::cout << "\033[1;41m \033[0m";
                else
                    std::cout << "X";
            else if (isTileBeingProcessed(i, j))
                if (drawUnixTerminalColors)
                    std::cout << "\033[1;43m \033[0m";
                else
                    std::cout << "V";
            else if (isTileVisited(i, j))
                if (drawUnixTerminalColors)
                    std::cout << "\033[1;42m \033[0m";
                else
                    std::cout << "P";
            else {
                std::pair<int,int> tileInd = std::make_pair(i, j) ;
                int tileBorderDataExists = m_mapTileToBorderVertices.count(tileInd);
                if (tileBorderDataExists)
                    if (drawUnixTerminalColors)
                        std::cout << "\033[1;46m \033[0m";
                    else
                        std::cout << "C";
                else
                if (drawUnixTerminalColors)
                    std::cout << "\033[1;47m \033[0m";
                else
                    std::cout << "·";
            }

        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
}