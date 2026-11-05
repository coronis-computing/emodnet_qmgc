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
    typedef TinCreation::Vector_3 Vector_3;

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
    // Normals (in ECEF) of the vertices above, so that both tiles sharing a border use the same ones
    std::vector<Vector_3> tileEastNormals; //!< Normals of tileEastVertices (same order)
    std::vector<Vector_3> tileWestNormals; //!< Normals of tileWestVertices (same order)
    std::vector<Vector_3> tileNorthNormals; //!< Normals of tileNorthVertices (same order)
    std::vector<Vector_3> tileSouthNormals; //!< Normals of tileSouthVertices (same order)
    Vector_3 northWestCornerNormal; //!< Normal of northWestCorner
    Vector_3 northEastCornerNormal; //!< Normal of northEastCorner
    Vector_3 southWestCornerNormal; //!< Normal of southWestCorner
    Vector_3 southEastCornerNormal; //!< Normal of southEastCorner
    // Whether each border is shared with an already built neighbor, and must thus be kept as it is. Note that the
    // vectors above may be empty even then, if the neighbor kept no vertices on that border other than its corners.
    bool constrainEasternBorder = false; //!< The eastern border must be maintained
    bool constrainWesternBorder = false; //!< The western border must be maintained
    bool constrainNorthernBorder = false; //!< The northern border must be maintained
    bool constrainSouthernBorder = false; //!< The southern border must be maintained

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
            , tileEastNormals()
            , tileWestNormals()
            , tileNorthNormals()
            , tileSouthNormals()
            , northWestCornerNormal(0, 0, 0)
            , northEastCornerNormal(0, 0, 0)
            , southWestCornerNormal(0, 0, 0)
            , southEastCornerNormal(0, 0, 0)
            , constrainNorthWestCorner(false)
            , constrainNorthEastCorner(false)
            , constrainSouthWestCorner(false)
            , constrainSouthEastCorner(false) {}

//    /**
//     * @brief Check if the north-west corner is already constrained by one of the borders
//     * @return true if the northern or western borders are not empty, since the corner should be one of the points in the vector (not checked though)
//     */
//    bool isNorthWestCornerConstrainedByBorders() {
//        return !tileNorthVertices.empty() || !tileWestVertices.empty();
//    }
//
//    /**
//     * @brief Check if the north-east corner is already constrained by one of the borders
//     * @return true if the northern or eastern borders are not empty, since the corner should be one of the points in the vector (not checked though)
//     */
//    bool isNorthEastCornerConstrainedByBorders() {
//        return !tileNorthVertices.empty() || !tileEastVertices.empty();
//    }
//
//    /**
//     * @brief Check if the south-west corner is already constrained by one of the borders
//     * @return true if the southern or western borders are not empty, since the corner should be one of the points in the vector (not checked though)
//     */
//    bool isSouthWestCornerConstrainedByBorders() {
//        return !tileSouthVertices.empty() || !tileWestVertices.empty();
//    }
//
//    /**
//     * @brief Check if the south-east corner is already constrained by one of the borders
//     * @return true if the southern or eastern borders are not empty, since the corner should be one of the points in the vector (not checked though)
//     */
//    bool isSouthEastCornerConstrainedByBorders() {
//        return !tileSouthVertices.empty() || !tileEastVertices.empty();
//    }

    /**
     * @brief Check wether north-west corner needs to be used
     * @return true if the validity flag for the north-west corner is set, and the northern-western borders are empty (otherwise, it will already be part of them)
     */
    bool useNorthWestCorner() {
        return constrainNorthWestCorner; // && !isNorthWestCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether north-east corner needs to be used
     * @return true if the validity flag for the north-east corner is set, and the northern-eastern borders are empty (otherwise, it will already be part of them)
     */
    bool useNorthEastCorner() {
        return constrainNorthEastCorner; // && !isNorthEastCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether south-west corner needs to be used
     * @return true if the validity flag for the south-west corner is set, and the southern-western borders are empty (otherwise, it will already be part of them)
     */
    bool useSouthWestCorner() {
        return constrainSouthWestCorner; // && !isSouthWestCornerConstrainedByBorders();
    }

    /**
     * @brief Check wether south-east corner needs to be used
     * @return true if the validity flag for the south-east corner is set, and the southern-eastern borders are empty (otherwise, it will already be part of them)
     */
    bool useSouthEastCorner() {
        return constrainSouthEastCorner; // && !isSouthEastCornerConstrainedByBorders();
    }

};

#endif //EMODNET_TOOLS_BORDERS_DATA_H
