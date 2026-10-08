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
// Derived from the Grid/GlobalGeodetic classes of cesium-terrain-builder
// (https://github.com/geo-data/cesium-terrain-builder),
// Copyright 2014 GeoData <geodata@soton.ac.uk>, licensed under the
// Apache License, Version 2.0.

#ifndef EMODNET_QMGC_TILING_GLOBAL_GEODETIC_GRID_H
#define EMODNET_QMGC_TILING_GLOBAL_GEODETIC_GRID_H

#include "geo_types.h"
#include <cmath>
#include <cstdint>

namespace qmgc {

/**
 * @class GlobalGeodeticGrid
 * @brief The TMS Global Geodetic profile (EPSG:4326).
 *
 * Models the [TMS Global Geodetic
 * Profile](http://wiki.osgeo.org/wiki/Tile_Map_Service_Specification#global-geodetic):
 * the world spans [-180, -90, 180, 90] degrees, the root level holds two tiles
 * side by side (2x1), and each zoom level doubles the number of tiles per axis.
 *
 * This collapses cesium-terrain-builder's Grid/GlobalGeodetic pair into a single
 * concrete class. The generic Grid base existed to also support GlobalMercator,
 * which this project never used. The spatial reference system is likewise not
 * stored: it is only needed when warping, where it is expressible as the string
 * "EPSG:4326".
 */
class GlobalGeodeticGrid {
public:
    /// A pixel index within a zoom level.
    ///
    /// Note: 64-bit, unlike cesium-terrain-builder's 32-bit `unsigned int`,
    /// which overflows beyond zoom level ~22.
    typedef std::int64_t i_pixel;

    /**
     * @brief Constructor
     * @param tileSize Number of samples along each side of a tile
     */
    explicit GlobalGeodeticGrid(i_pixel tileSize = 256)
            : m_tileSize(tileSize)
            , m_extent(-180.0, -90.0, 180.0, 90.0)
              // Two root tiles across 360 degrees
            , m_initialResolution((360.0 / 2.0) / static_cast<double>(tileSize))
            , m_xOriginShift(180.0)
            , m_yOriginShift(90.0) {}

    /// Get the number of samples along each side of a tile
    i_pixel tileSize() const { return m_tileSize; }

    /// Get the extent covered by the grid, in CRS coordinates
    const CRSBounds &getExtent() const { return m_extent; }

    /// Get the size of one sample, in degrees, at the given zoom level
    double resolution(i_zoom zoom) const {
        return m_initialResolution / std::pow(2.0, static_cast<double>(zoom));
    }

    /**
     * @brief Get the zoom level whose resolution best matches the one given
     *
     * If the resolution falls between two zoom levels it is rounded up to the
     * finer of the two.
     */
    i_zoom zoomForResolution(double resolution) const {
        return static_cast<i_zoom>(
                std::ceil(std::log2(m_initialResolution) - std::log2(resolution)));
    }

    /// Convert pixel coordinates at a given zoom level to CRS coordinates
    CRSPoint pixelsToCrs(const Coordinate<i_pixel> &pixel, i_zoom zoom) const {
        const double res = resolution(zoom);
        return CRSPoint((static_cast<double>(pixel.x) * res) - m_xOriginShift,
                        (static_cast<double>(pixel.y) * res) - m_yOriginShift);
    }

    /// Get the pixel location of a CRS coordinate at a given zoom level
    Coordinate<i_pixel> crsToPixels(const CRSPoint &coord, i_zoom zoom) const {
        const double res = resolution(zoom);
        return Coordinate<i_pixel>(static_cast<i_pixel>((m_xOriginShift + coord.x) / res),
                                   static_cast<i_pixel>((m_yOriginShift + coord.y) / res));
    }

    /// Get the tile containing a pixel location
    TilePoint pixelsToTile(const Coordinate<i_pixel> &pixel) const {
        return TilePoint(static_cast<i_tile>(pixel.x / m_tileSize),
                         static_cast<i_tile>(pixel.y / m_tileSize));
    }

    /**
     * @brief Get the tile in which a CRS coordinate falls at a given zoom level
     *
     * Note: a coordinate lying exactly on a tile's upper or right edge is
     * reported as belonging to the *next* tile along that axis. For a dataset
     * whose extent reaches exactly 180E or 90N this yields an upper-right tile
     * index one past the last real tile of the level.
     */
    TileCoordinate crsToTile(const CRSPoint &coord, i_zoom zoom) const {
        return TileCoordinate(zoom, pixelsToTile(crsToPixels(coord, zoom)));
    }

    /// Get the CRS extent covered by a tile
    CRSBounds tileBounds(const TileCoordinate &coord) const {
        const Coordinate<i_pixel> pxLowerLeft(static_cast<i_pixel>(coord.x) * m_tileSize,
                                              static_cast<i_pixel>(coord.y) * m_tileSize);
        const Coordinate<i_pixel> pxUpperRight((static_cast<i_pixel>(coord.x) + 1) * m_tileSize,
                                               (static_cast<i_pixel>(coord.y) + 1) * m_tileSize);

        return CRSBounds(pixelsToCrs(pxLowerLeft, coord.zoom),
                         pixelsToCrs(pxUpperRight, coord.zoom));
    }

    /// Get the tiles spanned by the whole grid at a given zoom level
    TileBounds getTileExtent(i_zoom zoom) const {
        const TileCoordinate ll = crsToTile(m_extent.getLowerLeft(), zoom);
        const TileCoordinate ur = crsToTile(m_extent.getUpperRight(), zoom);
        return TileBounds(ll, ur);
    }

private:
    i_pixel m_tileSize;          //!< Number of samples along each side of a tile
    CRSBounds m_extent;          //!< The extent covered by the grid
    double m_initialResolution;  //!< Size of one sample, in degrees, at zoom 0
    double m_xOriginShift;       //!< Shift to get to the origin from minX
    double m_yOriginShift;       //!< Shift to get to the origin from minY
};

} // namespace qmgc

#endif //EMODNET_QMGC_TILING_GLOBAL_GEODETIC_GRID_H
