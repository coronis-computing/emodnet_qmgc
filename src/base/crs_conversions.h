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

    void ecef2llh( const double& x, const double& y, const double& z,
                   double& lat, double& lon, double& h )
    {
        // Constants for the WGS84 Ellipsoid
        const double wgs84_a = 6378137.0;               // Semi-major axis
        const double wgs84_a2 = wgs84_a*wgs84_a;        // Semi-major axis squared
        const double wgs84_e2 = 6.6943799901377997e-3;  // First eccentricity squared
        const double wgs84_b = 6356752.3142451793;      // Semi-minor axis
        const double wgs84_b2 = wgs84_b*wgs84_b;        // Semi-minor axis squared

        double ep = sqrt((wgs84_a2 - wgs84_b2)/wgs84_b2);
        double p = sqrt(x*x + y*y);
        double th = atan2(wgs84_a*z, wgs84_b*p);
        lon = atan2(y, x);
        lat = atan2( z + (ep*ep) * wgs84_b * (sin(th)*sin(th)*sin(th)),
                     p - wgs84_e2 * wgs84_a * (cos(th)*cos(th)*cos(th)) );
        double N = wgs84_a/sqrt(1 - wgs84_e2 * (sin(lat)*sin(lat)));
        h = (p/cos(lat)) - N;

        // Radians to degrees
        lon *= 180./M_PI;
        lat *= 180./M_PI;
    }


};

#endif // EMODNET_TOOLS_CRS_CONVERSIONS_H
