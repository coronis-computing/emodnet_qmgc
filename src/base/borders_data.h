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

#ifndef EMODNET_QMGC_BORDERS_DATA_H
#define EMODNET_QMGC_BORDERS_DATA_H

#include "tin_creation/tin_creation_cgal_types.h"

/**
 * @class BordersData
 * @brief Structure storing the data for the borders of a tile. This includes the 4 east-west-north-south borders, as well as the corners.
 */
struct BordersData
{
    typedef TinCreation::Point_3 Point_3;

    std::vector<Point_3> tileEastVertices; //!< Vertices to maintain for the eastern border of the tile
    std::vector<Point_3> tileWestVertices; //!< Vertices to maintain for the western border of the tile
    std::vector<Point_3> tileNorthVertices; //!< Vertices to maintain for the northern border of the tile
    std::vector<Point_3> tileSouthVertices; //!< Vertices to maintain for the southern border of the tile
    bool constrainNorthWestCorner; //!< Flag indicating whether the north-west corner should be constrained
    bool constrainNorthEastCorner; //!< Flag indicating whether the north-east corner should be constrained
    bool constrainSouthWestCorner; //!< Flag indicating whether the south-west corner should be constrained
    bool constrainSouthEastCorner; //!< Flag indicating whether the north-east corner should be constrained
    Point_3 northWestCorner; //!< The north-west corner to maintain (if constrainNorthWestCorner is set)
    Point_3 northEastCorner; //!< The north-east corner to maintain (if constrainNorthEastCorner is set)
    Point_3 southWestCorner; //!< The south-west corner to maintain (if constrainSouthWestCorner is set)
    Point_3 southEastCorner; //!< The south-east corner to maintain (if constrainSouthEastCorner is set)

    /**
     * @brief Default constructor
     */
    BordersData()
            : tileEastVertices()
            , tileWestVertices()
            , tileNorthVertices()
            , tileSouthVertices()
            , northWestCorner(-1, -1, -1)
            , northEastCorner(-1, -1, -1)
            , southWestCorner(-1, -1, -1)
            , southEastCorner(-1, -1, -1)
            , constrainNorthWestCorner(false)
            , constrainNorthEastCorner(false)
            , constrainSouthWestCorner(false)
            , constrainSouthEastCorner(false) {}

    /**
     * @brief Copy constructor
     * @param bd BordersData structure to copy
     */
    BordersData(const BordersData& bd) {
        tileEastVertices = bd.tileEastVertices;
        tileWestVertices = bd.tileWestVertices;
        tileNorthVertices = bd.tileNorthVertices;
        tileSouthVertices = bd.tileSouthVertices;
        northWestCorner = bd.northWestCorner;
        northEastCorner = bd.northEastCorner;
        southWestCorner = bd.southWestCorner;
        southEastCorner = bd.southEastCorner;
        constrainNorthWestCorner = bd.constrainNorthWestCorner;
        constrainNorthEastCorner = bd.constrainNorthEastCorner;
        constrainSouthWestCorner = bd.constrainSouthWestCorner;
        constrainSouthEastCorner = bd.constrainSouthEastCorner;
    }

    /**
     * @brief Check if the north-west corner is already constrained by one of the borders
     * @return true if the northern or western borders are not empty, since the corner should be one of the points in the vector (not checked though)
     */
    bool isNorthWestCornerConstrainedByBorders() {
        return !tileNorthVertices.empty() || !tileWestVertices.empty();
    }

    /**
     * @brief Check if the north-east corner is already constrained by one of the borders
     * @return true if the northern or eastern borders are not empty, since the corner should be one of the points in the vector (not checked though)
     */
    bool isNorthEastCornerConstrainedByBorders() {
        return !tileNorthVertices.empty() || !tileEastVertices.empty();
    }

    /**
     * @brief Check if the south-west corner is already constrained by one of the borders
     * @return true if the southern or western borders are not empty, since the corner should be one of the points in the vector (not checked though)
     */
    bool isSouthWestCornerConstrainedByBorders() {
        return !tileSouthVertices.empty() || !tileWestVertices.empty();
    }

    /**
     * @brief Check if the south-east corner is already constrained by one of the borders
     * @return true if the southern or eastern borders are not empty, since the corner should be one of the points in the vector (not checked though)
     */
    bool isSouthEastCornerConstrainedByBorders() {
        return !tileSouthVertices.empty() || !tileEastVertices.empty();
    }

    /**
     * @brief Check wether north-west corner needs to be used
     * @return true if the validity flag for the north-west corner is set, and the northern-western borders are empty (otherwise, it will already be part of them)
     */
    bool useNorthWestCorner() {
        return constrainNorthWestCorner && !isNorthWestCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether north-east corner needs to be used
     * @return true if the validity flag for the north-east corner is set, and the northern-eastern borders are empty (otherwise, it will already be part of them)
     */
    bool useNorthEastCorner() {
        return constrainNorthEastCorner && !isNorthEastCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether south-west corner needs to be used
     * @return true if the validity flag for the south-west corner is set, and the southern-western borders are empty (otherwise, it will already be part of them)
     */
    bool useSouthWestCorner() {
        return constrainSouthWestCorner && !isSouthWestCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether south-east corner needs to be used
     * @return true if the validity flag for the south-east corner is set, and the southern-eastern borders are empty (otherwise, it will already be part of them)
     */
    bool useSouthEastCorner() {
        return constrainSouthEastCorner && !isSouthEastCornerConstrainedByBorders();
    }

};

#endif //EMODNET_TOOLS_BORDERS_DATA_H
