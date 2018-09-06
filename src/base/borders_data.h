//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_BORDERS_DATA_H
#define EMODNET_TOOLS_BORDERS_DATA_H

#include "tin_creation/tin_creation_cgal_types.h"

/**
 * @class
 * @brief Structure storing the data for the borders of a tile. This includes the 4 east-west-north-south borders, as well as the corners.
 */
struct BordersData
{
    typedef TinCreation::Point_3 Point_3;

    std::vector<Point_3> tileEastVertices;
    std::vector<Point_3> tileWestVertices;
    std::vector<Point_3> tileNorthVertices;
    std::vector<Point_3> tileSouthVertices;
    bool constrainNorthWestCorner;
    bool constrainNorthEastCorner;
    bool constrainSouthWestCorner;
    bool constrainSouthEastCorner;
    Point_3 northWestCorner;
    Point_3 northEastCorner;
    Point_3 southWestCorner;
    Point_3 southEastCorner;

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
