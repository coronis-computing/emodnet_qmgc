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
 * just store the variable coordinate and the height measure. The vertex normal (in ECEF) is also kept, so that the
 * neighboring tile can reuse it and the shading is continuous across the border.
 */
struct BorderVertex {
    double coord ;
    double height ;
    TinCreation::Vector_3 normal ;

    BorderVertex( const double& c, const double& h,
                  const TinCreation::Vector_3& n = TinCreation::Vector_3(0, 0, 0) )
            : coord(c), height(h), normal(n) {}

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
            : m_easternVerticesPtr()
            , m_westernVerticesPtr()
            , m_northernVerticesPtr()
            , m_southernVerticesPtr()
            , m_northWestCornerPtr()
            , m_southWestCornerPtr()
            , m_northEastCornerPtr()
            , m_southEastCornerPtr(){}

    bool hasEasternVertices() {return m_easternVerticesPtr != nullptr;}
    bool hasWesternVertices() {return m_westernVerticesPtr != nullptr;}
    bool hasNorthernVertices() {return m_northernVerticesPtr != nullptr;}
    bool hasSouthernVertices() {return m_southernVerticesPtr != nullptr;}
    bool hasNorthWestCorner() {return m_northWestCornerPtr!= nullptr;}
    bool hasSouthWestCorner() {return m_southWestCornerPtr!= nullptr;}
    bool hasNorthEastCorner() {return m_northEastCornerPtr!= nullptr;}
    bool hasSouthEastCorner() {return m_southEastCornerPtr!= nullptr;}

    // Note that, when setting, we only add another reference to the data already allocated in the vector
    void setEasternVertices(std::shared_ptr<std::vector<BorderVertex>> ptr) {m_easternVerticesPtr = ptr;}
    void setWesternVertices(std::shared_ptr<std::vector<BorderVertex>> ptr) {m_westernVerticesPtr = ptr;}
    void setNorthernVertices(std::shared_ptr<std::vector<BorderVertex>> ptr) {m_northernVerticesPtr = ptr;}
    void setSouthernVertices(std::shared_ptr<std::vector<BorderVertex>> ptr) {m_southernVerticesPtr = ptr;}
    void setNorthWestCorner(std::shared_ptr<BorderVertex> ptr) {m_northWestCornerPtr = ptr;}
    void setSouthWestCorner(std::shared_ptr<BorderVertex> ptr) {m_southWestCornerPtr = ptr;}
    void setNorthEastCorner(std::shared_ptr<BorderVertex> ptr) {m_northEastCornerPtr = ptr;}
    void setSouthEastCorner(std::shared_ptr<BorderVertex> ptr) {m_southEastCornerPtr = ptr;}

    std::vector<BorderVertex> getEasternVertices() {
        if (m_easternVerticesPtr == nullptr)
            return std::vector<BorderVertex>();
        else
            return *m_easternVerticesPtr;
    }
    std::vector<BorderVertex> getWesternVertices() {
        if (m_westernVerticesPtr == nullptr)
            return std::vector<BorderVertex>();
        else
            return *m_westernVerticesPtr;
    }
    std::vector<BorderVertex> getNorthernVertices() {
        if (m_northernVerticesPtr == nullptr)
            return std::vector<BorderVertex>();
        else
            return *m_northernVerticesPtr;
    }
    std::vector<BorderVertex> getSouthernVertices() {
        if (m_southernVerticesPtr == nullptr)
            return std::vector<BorderVertex>();
        else
            return *m_southernVerticesPtr;
    }
    BorderVertex getNorthWestCorner() {return *m_northWestCornerPtr;}
    BorderVertex getSouthWestCorner() {return *m_southWestCornerPtr;}
    BorderVertex getNorthEastCorner() {return *m_northEastCornerPtr;}
    BorderVertex getSouthEastCorner() {return *m_southEastCornerPtr;}

    std::shared_ptr<std::vector<BorderVertex>> getEasternVerticesPtr() {return m_easternVerticesPtr;}
    std::shared_ptr<std::vector<BorderVertex>> getWesternVerticesPtr() {return m_westernVerticesPtr;}
    std::shared_ptr<std::vector<BorderVertex>> getNorthernVerticesPtr() {return m_northernVerticesPtr;}
    std::shared_ptr<std::vector<BorderVertex>> getSouthernVerticesPtr() {return m_southernVerticesPtr;}
    std::shared_ptr<BorderVertex> getNorthWestCornerPtr() {return m_northWestCornerPtr;}
    std::shared_ptr<BorderVertex> getSouthWestCornerPtr() {return m_southWestCornerPtr;}
    std::shared_ptr<BorderVertex> getNorthEastCornerPtr() {return m_northEastCornerPtr;}
    std::shared_ptr<BorderVertex> getSouthEastCornerPtr() {return m_southEastCornerPtr;}

private:
    std::shared_ptr<std::vector<BorderVertex>> m_easternVerticesPtr;
    std::shared_ptr<std::vector<BorderVertex>> m_westernVerticesPtr;
    std::shared_ptr<std::vector<BorderVertex>> m_northernVerticesPtr;
    std::shared_ptr<std::vector<BorderVertex>> m_southernVerticesPtr;
    // Note that for corners only the height and normal are meaningful (the coord is unused)
    std::shared_ptr<BorderVertex> m_northWestCornerPtr;
    std::shared_ptr<BorderVertex> m_southWestCornerPtr;
    std::shared_ptr<BorderVertex> m_northEastCornerPtr;
    std::shared_ptr<BorderVertex> m_southEastCornerPtr;
};

#endif //EMODNET_QMGC_TILE_BORDER_VERTICES_H
