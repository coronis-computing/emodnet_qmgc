//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_CRS_CONVERSIONS_H
#define EMODNET_TOOLS_CRS_CONVERSIONS_H

#include <math.h>

// Coordinate Reference System conversions
namespace crs_conversions {

    void llh2ecef( const double& lat, const double& lon, const double& h,
                   double& x, double& y, double& z )
    {
        // Constants for the WGS84 Ellipsoid
        const double wgs84_a = 6378137.0;               // Semi-major axis
        const double wgs84_e2 = 6.6943799901377997e-3;  // First eccentricity squared

        // First convert lat/lon to radians
        double latRad = lat*(M_PI/180.0);
        double lonRad = lon*(M_PI/180.0);

        double n = wgs84_a/sqrt( 1 - wgs84_e2*sin( latRad )*sin( latRad ) );
        x = ( n + h )*cos( latRad )*cos( lonRad );        // ECEF x
        y = ( n + h )*cos( latRad )*sin( lonRad );        // ECEF y
        z = ( n*(1 - wgs84_e2 ) + h )*sin( latRad );   //ECEF z
    }

};

#endif // EMODNET_TOOLS_CRS_CONVERSIONS_H
