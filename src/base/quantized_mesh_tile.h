//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_QUANTIZEDMESHTILE_H
#define EMODNET_TOOLS_QUANTIZEDMESHTILE_H

#include "quantized_mesh.h"
#include "ellipsoid.h"
#include "tin_creation/tin_creation_cgal_types.h"


/**
 * @class
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

    /// Create a quantized mesh tile from a tile coordinate
    QuantizedMeshTile(const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh()
            , Tile(coord)
            , m_ellipsoid(e) {}

    /// Create a quantized mesh tile from a file
    QuantizedMeshTile(const char *fileName,
                      const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh(fileName)
            , Tile(coord)
            , m_ellipsoid(e) {}


    /// Create a quantized mesh tile from quantized mesh data
    QuantizedMeshTile(const QuantizedMesh &qm,
                      const ctb::TileCoordinate &coord,
                      const Ellipsoid& e = WGS84Ellipsoid())
            : QuantizedMesh(qm)
            , Tile(coord)
            , m_ellipsoid(e) {}

    /// Convert U/V/Height coordinates in quantized mesh tile to Lon/Lat/Height
    void convertUVHToLonLatHeight( const unsigned short &u, const unsigned short &v, const unsigned short &h,
                                   double &lon, double &lat, double &height ) ;

    /// Export to OFF format
    bool exportToOFF( const std::string &outFilePath, const bool& useRealWorldValues = false ) ;

    /**
     * \brief Compute the horizon occlusion point for the given tile
     *
     * \p pts Points are supposed to be in ECEF coordinates
     */
    Point_3 horizonOcclusionPoint( const std::vector<Point_3> &pts, const Point_3 &center ) ;


private:
    // Function as described in https://cesium.com/blog/2013/05/09/computing-the-horizon-occlusion-point/
    // We ommit the ellipsoid here for simplicity (hard-coded)
    double computeHorizonOcclusionPointMagnitude( const Point_3 &position, const Vector_3 &scaledSpaceDirectionToPoint ) ;


    // --- Attributes ---
    Ellipsoid m_ellipsoid ; /// The reference ellipsoid
};

#endif //EMODNET_TOOLS_QUANTIZEDMESHTILE_H
