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

#ifndef EMODNET_QMGC_CRS_CONVERSIONS_H
#define EMODNET_QMGC_CRS_CONVERSIONS_H

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

#endif // EMODNET_QMGC_CONVERSIONS_H
