//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_ELLIPSOID_H
#define EMODNET_TOOLS_ELLIPSOID_H

/**
 * @class
 *
 * @brief Describes an ellipsoid.
 *
 * A quadratic surface of the form (x / rx)^2 + (y / ry)^2 + (z / rz)^2 = 1.
 * The ellipsoid chosen has implications in the Quantized Mesh format, specially for computing the occlusion point of a
 * Quantized Mesh tile.
 *
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

    /// Copy constructor
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
    double m_rx, m_ry, m_rz ; /// Radius in X, Y and Z defining the ellipsoid
};



/**
 * @class
 *
 * @brief The WGS84 ellipsoid
 *
 * An instance of the WGS84 ellipsoid
 */
class WGS84Ellipsoid : public Ellipsoid
{
public:
    WGS84Ellipsoid() : Ellipsoid(6378137.0, 6378137.0, 6356752.3142451793) {}
};

#endif //EMODNET_TOOLS_ELLIPSOID_H
