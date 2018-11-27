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

#ifndef EMODNET_QMGC_TIN_CREATOR_H
#define EMODNET_QMGC_TIN_CREATOR_H

#include <memory>
#include "tin_creation_cgal_types.h"
#include "base/misc_utils.h"

// Note: this set of classes implement a Strategy Pattern
namespace TinCreation {

/**
 * @class TinCreationStrategy
 * @brief Defines the interphase of a TIN creation algorithm
 *
 * (Interphase class of a Strategy Pattern)
 */
class TinCreationStrategy {
public:
    /// Constructor
    TinCreationStrategy()
            : m_scaleZ(1.0)
            , m_minX(-1.0), m_minY(-1.0), m_minZ(-1.0), m_maxX(-1.0), m_maxY(-1.0), m_maxZ(-1.0)
            , m_boundsSet(false) {}

    /**
    * @brief Create a TIN from a set of points.
    *
    * While it does not impose any regularity on the input points, the use of constrain<X>Vertices is restricted to the
    * shape of the convex hull of the input point being an axis-aligned rectangle.
    *
    * @param dataPts Input 3D point set
    * @param constrainEasternVertices Flag indicating whether the vertices on the eastern border of the tile should be preserved or not
    * @param constrainWesternVertices Flag indicating whether the vertices on the western border of the tile should be preserved or not
    * @param constrainNorthernVertices Flag indicating whether the vertices on the northern border of the tile should be preserved or not
    * @param constrainSouthernVertices Flag indicating whether the vertices on the southern border of the tile should be preserved or not
    * @return
    */
    virtual Polyhedron create(const std::vector<Point_3> &dataPts,
                              const bool &constrainEasternVertices,
                              const bool &constrainWesternVertices,
                              const bool &constrainNorthernVertices,
                              const bool &constrainSouthernVertices) = 0;

    /**
     * @brief Adapts the parameters of the algorithm for the desired zoom level.
     *
     * For further information, see the parameters' setting mechanism in the tutorial.
     *
     * @param zoom Current zoom level
     */
    virtual void setParamsForZoom(const unsigned int& zoom) = 0;

    /**
     * Set the scale in Z (used by some of the methods to scale the parameters w.r.t. the tile units)
     * @param scale Scale in Z
     */
    void setScaleZ(const double& scale) { m_scaleZ = scale; }

    /// Get the scale in Z
    double getScaleZ() { return m_scaleZ; }

    /**
     * Set the geographic bounds for the current tile (used by some of the methods to scale the parameters w.r.t. the tile units)
     * @param minX Minimum X value
     * @param minY Minimum Y value
     * @param minZ Minimum Z value
     * @param maxX Maximum X value
     * @param maxY Maximum Y value
     * @param maxZ Maximum Z value
     */
    void setBounds(const double& minX, const double& minY, const double& minZ,
                   const double& maxX, const double& maxY, const double& maxZ ) {
        m_minX = minX; m_minY = minY; m_minZ = minZ;
        m_maxX = maxX; m_maxY = maxY; m_maxZ = maxZ;
        m_boundsSet = true;
    }

    /// Get the minimum X coordinate
    double getMinX() const { return m_minX; }
    /// Get the minimum Y coordinate
    double getMinY() const { return m_minY; }
    /// Get the minimum Z coordinate
    double getMinZ() const { return m_minZ; }
    /// Get the maximum X coordinate
    double getMaxX() const { return m_maxX; }
    /// Get the maximum Y coordinate
    double getMaxY() const { return m_maxY; }
    /// Get the maximum Z coordinate
    double getMaxZ() const { return m_maxZ; }

    bool hasOriginalBoundingBox() const { return m_boundsSet; }

    /// Convert points to ECEF assuming that they are on a UVH format, and given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    std::vector<Point_3> convertUVHToECEF(const std::vector<Point_3>& pts) const;

    /// Convert points from local UVH to ECEF given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    std::vector<Point_3> convertECEFToUVH(const std::vector<Point_3>& pts) const;

    /// Convert points to ECEF assuming that they are on a UVH format, and given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    Point_3 convertUVHToECEF(const Point_3& p) const;

    /// Convert points from local UVH to ECEF given the limits of the tile
    // This conversion is used by some point set simplification methods requiring metric coordinates
    Point_3 convertECEFToUVH(const Point_3& p) const;

private:
    double m_scaleZ;
    double m_minX, m_minY, m_minZ, m_maxX, m_maxY, m_maxZ;
    bool m_boundsSet;
};

/**
 * @class TinCreator
 * @brief Main class used to create a TIN from an input set of points
 *
 * This class does not implement any particular method, the concrete strategy to use should be set using the setCreator() method.
 *
 * (Context class of a Strategy Pattern)
 */
class TinCreator {
public:
    // --- Methods ---
    /// Constructor
    TinCreator() { m_creator = nullptr; }

    /**
     * @brief Sets the actual TIN creator algorithm
     * @param creator Pointer to the actual TIN creator algorithm to use
     */
    void setCreator(std::shared_ptr<TinCreationStrategy> creator) { m_creator = creator; }

    /**
     * @brief Create a TIN from a set of points.
     *
     * While it does not impose any regularity on the input points, the use of constrain<X>Vertices is restricted to the
     * shape of the convex hull of the input point being an axis-aligned rectangle.
     *
     * @param dataPts Input 3D point set
     * @param constrainEasternVertices Flag indicating whether the vertices on the eastern border of the tile should be preserved or not
     * @param constrainWesternVertices Flag indicating whether the vertices on the western border of the tile should be preserved or not
     * @param constrainNorthernVertices Flag indicating whether the vertices on the northern border of the tile should be preserved or not
     * @param constrainSouthernVertices Flag indicating whether the vertices on the southern border of the tile should be preserved or not
     * @return
     */
    Polyhedron create(const std::vector<Point_3> &dataPts,
                      const bool &constrainEasternVertices = false,
                      const bool &constrainWesternVertices = false,
                      const bool &constrainNorthernVertices = false,
                      const bool &constrainSouthernVertices = false) {
        return m_creator->create(dataPts,
                                 constrainEasternVertices,
                                 constrainWesternVertices,
                                 constrainNorthernVertices,
                                 constrainSouthernVertices);
    }

    /**
     * @brief Adapts the parameters of the algorithm for the desired zoom level.
     *
     * For further information, see the parameters' setting mechanism in the tutorial.
     *
     * @param zoom Current zoom level
     */
    void setParamsForZoom(const unsigned int& zoom) {
        m_creator->setParamsForZoom(zoom);
    }

    /**
     * Set the scale in Z (used by some of the methods to scale the parameters w.r.t. the tile units)
     * @param scale Scale in Z
     */
    void setScaleZ(const double& scale) { m_creator->setScaleZ(scale); }

    /**
     * Set the geographic bounds for the current tile (used by some of the methods to scale the parameters w.r.t. the tile units)
     * @param minX Minimum X value
     * @param minY Minimum Y value
     * @param minZ Minimum Z value
     * @param maxX Maximum X value
     * @param maxY Maximum Y value
     * @param maxZ Maximum Z value
     */
    void setBounds(const double& minX, const double& minY, const double& minZ,
                   const double& maxX, const double& maxY, const double& maxZ) {
        m_creator->setBounds(minX, minY, minZ, maxX, maxY, maxZ);
        double scaleZ = remap( 1.0, 0.0, maxZ-minZ, 0.0, 1.0 );
        setScaleZ(scaleZ);
    }

private:
    // --- Attributes ---
    std::shared_ptr<TinCreationStrategy> m_creator;
};

} // End namespace tin_creation

#endif //EMODNET_QMGC_TIN_CREATOR_H
