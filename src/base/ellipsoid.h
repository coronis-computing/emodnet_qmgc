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

#ifndef EMODNET_QMGC_ELLIPSOID_H
#define EMODNET_QMGC_ELLIPSOID_H

/**
 * @class Ellipsoid
 * @brief Describes an ellipsoid.
 *
 * A quadratic surface of the form (x / rx)^2 + (y / ry)^2 + (z / rz)^2 = 1.
 * The ellipsoid chosen has implications in the Quantized Mesh format, specially for computing the occlusion point of a
 * Quantized Mesh tile.
 */
class Ellipsoid
{
public:
    /**
     * @brief Constructor
     * @param rx Radius of the ellipsoid in X
     * @param ry Radius of the ellipsoid in Y
     * @param rz Radius of the ellipsoid in Z
     */
    Ellipsoid( const double& rx, const double& ry, const double& rz ) : m_rx(rx), m_ry(ry), m_rz(rz) {}

    /// Default constructor (all radius set to 0)
    Ellipsoid() : m_rx(0.0), m_ry(0.0), m_rz(0.0) {}

    /**
     * @brief Copy constructor
     * @param e The ellipsoid to copy
     */
    Ellipsoid( const Ellipsoid& e ) {
        m_rx = e.getRadiusX() ;
        m_ry = e.getRadiusY() ;
        m_rz = e.getRadiusZ() ;
    }

    /// Get the X radius of the ellipsoid
    double getRadiusX() const { return m_rx ; }

    /// Get the Y radius of the ellipsoid
    double getRadiusY() const { return m_ry ; }

    /// Get the Z radius of the ellipsoid
    double getRadiusZ() const { return m_rz ; }

    /// Set the X radius of the ellipsoid
    void setRadiusX(const double& rx) { m_rx = rx ; }

    /// Set the Y radius of the ellipsoid
    void setRadiusY(const double& ry) { m_ry = ry ; }

    /// Set the Z radius of the ellipsoid
    void setRadiusZ(const double& rz) { m_rz = rz ; }

protected:
    double m_rx; //!< Radius of the ellipsoid in X
    double m_ry; //!< Radius of the ellipsoid in Y
    double m_rz; //!< Radius of the ellipsoid in Z
};

/**
 * @class WGS84Ellipsoid
 * @brief The WGS84 ellipsoid
 *
 * An instance of the WGS84 ellipsoid
 */
class WGS84Ellipsoid : public Ellipsoid
{
public:
    /**
     * @brief Constructor
     */
    WGS84Ellipsoid() : Ellipsoid(6378137.0, 6378137.0, 6356752.3142451793) {}
};

#endif //EMODNET_QMGC_ELLIPSOID_H
