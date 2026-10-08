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

#include "crs_conversions.h"

namespace crs_conversions
{

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
    // --- Simple method ---
//    // Constants for the WGS84 Ellipsoid
//    const double wgs84_a = 6378137.0;               // Semi-major axis
//    const double wgs84_a2 = wgs84_a*wgs84_a;        // Semi-major axis squared
//    const double wgs84_e2 = 6.6943799901377997e-3;  // First eccentricity squared
//    const double wgs84_b = 6356752.3142451793;      // Semi-minor axis
//    const double wgs84_b2 = wgs84_b*wgs84_b;        // Semi-minor axis squared
//
//    double ep = sqrt((wgs84_a2 - wgs84_b2)/wgs84_b2);
//    double p = sqrt(x*x + y*y);
//    double th = atan2(wgs84_a*z, wgs84_b*p);
//    lon = atan2(y, x);
//    lat = atan2( z + (ep*ep) * wgs84_b * (sin(th)*sin(th)*sin(th)),
//                 p - wgs84_e2 * wgs84_a * (cos(th)*cos(th)*cos(th)) );
//    double N = wgs84_a/sqrt(1 - wgs84_e2 * (sin(lat)*sin(lat)));
//    h = (p/cos(lat)) - N;

    // --- Olson ---
    // Implements: (Olson, D. K. (1996). "Converting earth-Centered, Earth-Fixed Coordinates to Geodetic Coordinates," IEEE Transactions on Aerospace and Electronic Systems, Vol. 32, No. 1, January 1996, pp. 473-476
    // Code extracted from: http://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html

    const double  a = 6378137.0;              //WGS-84 semi-major axis
    const double e2 = 6.6943799901377997e-3;  //WGS-84 first eccentricity squared
    const double a1 = 4.2697672707157535e+4;  //a1 = a*e2
    const double a2 = 1.8230912546075455e+9;  //a2 = a1*a1
    const double a3 = 1.4291722289812413e+2;  //a3 = a1*e2/2
    const double a4 = 4.5577281365188637e+9;  //a4 = 2.5*a2
    const double a5 = 4.2840589930055659e+4;  //a5 = a1+a3
    const double a6 = 9.9330562000986220e-1;  //a6 = 1-e2

    double zp = fabs(z);
    double w2 = x*x + y*y;
    double w = sqrt(w2);
    double r2 = w2 + z*z;
    double r = sqrt( r2 );
    lon = atan2( y, x );       //Lon (final)
    double s2 = z*z/r2;
    double c2 = w2/r2;
    double u = a2/r;
    double v = a3 - a4/r;
    double c, ss, s;
    if( c2 > 0.3 ){
        s = ( zp/r )*( 1.0 + c2*( a1 + u + s2*v )/r );
        lat = asin( s );      //Lat
        double ss = s*s;
        c = sqrt( 1.0 - ss );
    }
    else{
        c = ( w/r )*( 1.0 - s2*( a5 - u - c2*v )/r );
        lat = acos( c );      //Lat
        ss = 1.0 - c*c;
        s = sqrt( ss );
    }
    double g = 1.0 - e2*ss;
    double rg = a/sqrt( g );
    double rf = a6*rg;
    u = w - rg*c;
    v = zp - rf*s;
    double f = c*u + s*v;
    double m = c*v - s*u;
    double p = m/( rf/g + f );
    lat = lat + p;      //Lat
    h = f + m*p/2.0;     //Altitude
    if( z < 0.0 ){
        lat *= -1.0;     //Lat
    }

    // Radians to degrees
    lon *= 180./M_PI;
    lat *= 180./M_PI;
}

} // End namespace crs_conversions