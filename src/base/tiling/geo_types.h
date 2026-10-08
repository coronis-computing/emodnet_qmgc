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
//
// Derived from the Coordinate/Bounds/TileCoordinate classes of
// cesium-terrain-builder (https://github.com/geo-data/cesium-terrain-builder),
// Copyright 2014 GeoData <geodata@soton.ac.uk>, licensed under the
// Apache License, Version 2.0.

#ifndef EMODNET_QMGC_TILING_GEO_TYPES_H
#define EMODNET_QMGC_TILING_GEO_TYPES_H

#include "qmgc_exception.h"

namespace qmgc {

/// A tile index along one axis.
///
/// Note: deliberately *signed*, unlike cesium-terrain-builder's `unsigned int`.
/// The tile schedulers and the border-vertices cache probe neighbours with
/// expressions such as `tx-1`, and compare the result against the zoom bounds.
/// With an unsigned type, `tx = -1` wraps to 4294967295 and passes a
/// `>= getMinX()` test, so out-of-bounds neighbours are silently accepted.
typedef int i_tile;

/// A zoom level.
typedef int i_zoom;

/**
 * @class Coordinate
 * @brief A point in a two-dimensional space.
 */
template<class T>
struct Coordinate {
    /// Create a coordinate at the origin
    Coordinate() : x(0), y(0) {}

    /// Instantiate a coordinate from an x and y value
    Coordinate(T x, T y) : x(x), y(y) {}

    bool operator==(const Coordinate &other) const { return x == other.x && y == other.y; }
    bool operator!=(const Coordinate &other) const { return !(*this == other); }

    T x; //!< The x coordinate
    T y; //!< The y coordinate
};

/// The location of a tile within a zoom level
typedef Coordinate<i_tile> TilePoint;

/// A coordinate in the grid's coordinate reference system (EPSG:4326 degrees)
typedef Coordinate<double> CRSPoint;

/**
 * @class Bounds
 * @brief A two-dimensional extent, stored as { minX, minY, maxX, maxY }.
 *
 * The extent is validated on construction and on every mutation: a minimum
 * greater than its corresponding maximum throws a QmgcException.
 */
template<class T>
class Bounds {
public:
    /// Create a degenerate extent at the origin
    Bounds() : m_bounds{0, 0, 0, 0} {}

    /// Create an extent from its four edges
    Bounds(T minX, T minY, T maxX, T maxY) { setBounds(minX, minY, maxX, maxY); }

    /// Create an extent from its lower-left and upper-right corners
    Bounds(const Coordinate<T> &lowerLeft, const Coordinate<T> &upperRight) {
        setBounds(lowerLeft, upperRight);
    }

    bool operator==(const Bounds<T> &other) const {
        return m_bounds[0] == other.m_bounds[0] && m_bounds[1] == other.m_bounds[1] &&
               m_bounds[2] == other.m_bounds[2] && m_bounds[3] == other.m_bounds[3];
    }

    /// Set the extent from its four edges
    void setBounds(T minX, T minY, T maxX, T maxY) {
        if (minX > maxX)
            throw QmgcException("The minimum X value is greater than the maximum X value");
        if (minY > maxY)
            throw QmgcException("The minimum Y value is greater than the maximum Y value");

        m_bounds[0] = minX;
        m_bounds[1] = minY;
        m_bounds[2] = maxX;
        m_bounds[3] = maxY;
    }

    /// Set the extent from its lower-left and upper-right corners
    void setBounds(const Coordinate<T> &lowerLeft, const Coordinate<T> &upperRight) {
        setBounds(lowerLeft.x, lowerLeft.y, upperRight.x, upperRight.y);
    }

    T getMinX() const { return m_bounds[0]; } //!< Get the minimum X value
    T getMinY() const { return m_bounds[1]; } //!< Get the minimum Y value
    T getMaxX() const { return m_bounds[2]; } //!< Get the maximum X value
    T getMaxY() const { return m_bounds[3]; } //!< Get the maximum Y value

    /// Get the lower-left corner
    Coordinate<T> getLowerLeft() const { return Coordinate<T>(getMinX(), getMinY()); }

    /// Get the upper-right corner
    Coordinate<T> getUpperRight() const { return Coordinate<T>(getMaxX(), getMaxY()); }

    /// Get the extent's width
    T getWidth() const { return getMaxX() - getMinX(); }

    /// Get the extent's height
    T getHeight() const { return getMaxY() - getMinY(); }

private:
    T m_bounds[4]; //!< The extent, as { minX, minY, maxX, maxY }
};

/// Tile extents in tile coordinates (inclusive on all four sides)
typedef Bounds<i_tile> TileBounds;

/// Extents in the grid's coordinate reference system (EPSG:4326 degrees)
typedef Bounds<double> CRSBounds;

/**
 * @class TileCoordinate
 * @brief Identifies a single tile: a tile point plus the zoom level it lives in.
 */
class TileCoordinate : public TilePoint {
public:
    /// Create the 0/0/0 tile coordinate
    TileCoordinate() : TilePoint(0, 0), zoom(0) {}

    /// Instantiate a tile coordinate from a zoom level and an x/y index
    TileCoordinate(i_zoom zoom, i_tile x, i_tile y) : TilePoint(x, y), zoom(zoom) {}

    /// Instantiate a tile coordinate from a zoom level and a tile point
    TileCoordinate(i_zoom zoom, const TilePoint &coord) : TilePoint(coord), zoom(zoom) {}

    bool operator==(const TileCoordinate &other) const {
        return TilePoint::operator==(other) && zoom == other.zoom;
    }

    /// Set the tile point, leaving the zoom level untouched
    void setPoint(const TilePoint &point) { TilePoint::operator=(point); }

    i_zoom zoom; //!< The zoom level
};

} // namespace qmgc

#endif //EMODNET_QMGC_TILING_GEO_TYPES_H
