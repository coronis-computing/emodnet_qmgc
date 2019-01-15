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

#ifndef EMODNET_QMGC_TILE_BORDER_VERTICES_H
#define EMODNET_QMGC_TILE_BORDER_VERTICES_H

#include "tin_creation/tin_creation_cgal_types.h"
#include <vector>


/**
 * @struct BorderVertex
 * @brief Class storing a vertex on the border of a tile
 *
 * Since for the border vertices one of the coordinates can be deduced depending on which border vertex they are, we
 * just store the variable coordinate and the height measure
 */
struct BorderVertex {
    double coord ;
    double height ;

    BorderVertex( const double& c, const double& h ) : coord(c), height(h) {}

    // To be able to sort the vertices by coordinates
    bool operator < (const BorderVertex& v) const
    {
        return (coord < v.coord);
    }
};


/**
 * @class TileBorderVertices
 * @brief Stores the vertices on the borders of a tile.
 */
class TileBorderVertices
{
public:
    TileBorderVertices()
            : m_easternVertices()
            , m_westernVertices()
            , m_northernVertices()
            , m_southernVertices()
            , m_lifeCounter(0) {}

    TileBorderVertices( const std::vector<BorderVertex>& easternVertices,
                        const std::vector<BorderVertex>& westernVertices,
                        const std::vector<BorderVertex>& northernVertices,
                        const std::vector<BorderVertex>& southernVertices,
                        const int& lifeCounter = 4 ) // The life counter will be 4 for those tiles not in the zoom borders and surrounded by unprocessed tiles
            : m_easternVertices(easternVertices)
            , m_westernVertices(westernVertices)
            , m_northernVertices(northernVertices)
            , m_southernVertices(southernVertices)
            , m_lifeCounter(lifeCounter) {}

    // Each time we consult for a border, we update the number of times it has been consulted by decreasing the lifeCounter.
    // When it gets to zero, the information in this object will not be needed anymore and can be deleted (responsability of the cache object)
    std::vector<BorderVertex> getEasternVerticesAndDecreaseLife() { m_lifeCounter-- ; return m_easternVertices ; }
    std::vector<BorderVertex> getWesternVerticesAndDecreaseLife() { m_lifeCounter-- ; return m_westernVertices ; }
    std::vector<BorderVertex> getNorthernVerticesAndDecreaseLife() { m_lifeCounter-- ; return m_northernVertices ; }
    std::vector<BorderVertex> getSouthernVerticesAndDecreaseLife() { m_lifeCounter-- ; return m_southernVertices ; }

    std::vector<BorderVertex> getEasternVertices() { return m_easternVertices ; }
    std::vector<BorderVertex> getWesternVertices() { return m_westernVertices ; }
    std::vector<BorderVertex> getNorthernVertices() { return m_northernVertices ; }
    std::vector<BorderVertex> getSouthernVertices() { return m_southernVertices ; }

    void decreaseLife() { m_lifeCounter--; }

    bool isAlive() { return m_lifeCounter > 0 ; }

private:
    std::vector<BorderVertex> m_easternVertices ;
    std::vector<BorderVertex> m_westernVertices ;
    std::vector<BorderVertex> m_northernVertices ;
    std::vector<BorderVertex> m_southernVertices ;
    int m_lifeCounter ;
};

#endif //EMODNET_QMGC_TILE_BORDER_VERTICES_H
