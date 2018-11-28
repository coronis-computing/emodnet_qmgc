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

#ifndef EMODNET_QMGC_QUANTIZED_MESH_TILE_H
#define EMODNET_QMGC_QUANTIZED_MESH_TILE_H

#include "quantized_mesh.h"
#include "ellipsoid.h"
#include "tin_creation/tin_creation_cgal_types.h"


/**
 * @class QuantizedMeshTile
 * @brief Encapsulates a Tile in Quantized Mesh format *
 */
class QuantizedMeshTile :
public QuantizedMesh, public ctb::Tile
{
    friend class TerrainTiler;
    friend class QuantizedMeshTiler;

public:
    // --- Typedefs ---
    typedef TinCreation::Point_3   Point_3 ;
    typedef TinCreation::Vector_3  Vector_3 ;

    /**
     * @brief Creates a quantized mesh tile from a tile coordinate
     * @param coord Coordinates of the tile
     * @param e The reference Ellipsoid to use
     */
    QuantizedMeshTile(const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh()
            , Tile(coord)
            , m_ellipsoid(e) {}

    /**
     * @brief Create a quantized mesh tile from a file
     * @param fileName The quantized-mesh tile file
     * @param coord Coordinates of the tile
     * @param e The reference Ellipsoid to use
     */
    QuantizedMeshTile(const char *fileName,
                      const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh(fileName)
            , Tile(coord)
            , m_ellipsoid(e) {}

    /**
     * @brief Creates a quantized mesh tile from quantized mesh data
     * @param qm QuantizedMesh data
     * @param coord Coordinates of the tile
     * @param e The reference Ellipsoid to use
     */
    QuantizedMeshTile(const QuantizedMesh &qm,
                      const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh(qm)
            , Tile(coord)
            , m_ellipsoid(e) {}

    /// Convert U/V/Height coordinates in quantized mesh tile to Lon/Lat/Height
    void convertUVHToLonLatHeight( const unsigned short &u, const unsigned short &v, const unsigned short &h,
                                   double &lon, double &lat, double &height ) const ;

    /**
     * @brief Exports the geometry of the tile to OFF format
     * @param outFilePath Output file path
     * @param useRealWorldValues Flag indicating whether the vertices of the output geometry are the real values (i.e., the lat/lon/height coordinates) or the quantized values.
     * @return True if file created successfully.
     */
    bool exportToOFF( const std::string &outFilePath, const bool& useRealWorldValues = false ) ;

    /**
     * @brief Compute the horizon occlusion point for the given tile
     * @param pts Vertices of the tile (in ECEF coordinates!)
     * @param center Centroid of the tile
     * @return Horizon occlusion point
     */
    Point_3 horizonOcclusionPoint( const std::vector<Point_3> &pts, const Point_3 &center ) ;

private:
    // Function as described in https://cesium.com/blog/2013/05/09/computing-the-horizon-occlusion-point/
    // We ommit the ellipsoid here for simplicity (hard-coded)
    double computeHorizonOcclusionPointMagnitude( const Point_3 &position, const Vector_3 &scaledSpaceDirectionToPoint ) ;

    // --- Attributes ---
    Ellipsoid m_ellipsoid ; //!< The reference ellipsoid
};

#endif //EMODNET_QMGC_QUANTIZED_MESH_TILE_H
