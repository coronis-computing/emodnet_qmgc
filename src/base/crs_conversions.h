//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_CRS_CONVERSIONS_H
#define EMODNET_TOOLS_CRS_CONVERSIONS_H

#include <math.h>

// Coordinate Reference System conversions
namespace crs_conversions
{
    /**
     * @brief Converts from Latitude Longitude to Earth-Centered Earth-Fixed coordinates
     * @param lat Latitude (in degrees)
     * @param lon Longitude (in degrees)
     * @param h Height (in meters)
     * @param x Earth-Centered Earth-Fixed X
     * @param y Earth-Centered Earth-Fixed Y
     * @param z Earth-Centered Earth-Fixed Z
     */
    void llh2ecef( const double& lat, const double& lon, const double& h,
                   double& x, double& y, double& z );

    /**
     * @brief Converts from Earth-Centered Earth-Fixed to Latitude Longitude coordinates
     * @param x Earth-Centered Earth-Fixed X
     * @param y Earth-Centered Earth-Fixed Y
     * @param z Earth-Centered Earth-Fixed Z
     * @param lat Latitude (in degrees)
     * @param lon Longitude (in degrees)
     * @param h Height (in meters)
     */
    void ecef2llh( const double& x, const double& y, const double& z,
                   double& lat, double& lon, double& h );
} // End namespace crs_conversions

#endif // EMODNET_TOOLS_CRS_CONVERSIONS_H
