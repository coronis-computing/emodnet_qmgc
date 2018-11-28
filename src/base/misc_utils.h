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

#ifndef EMODNET_QMGC_MISC_UTILS_H
#define EMODNET_QMGC_MISC_UTILS_H

/**
 * Remapping function. Changes a value in the range \p minOr \p maxOr to a value in the range \p minDest \p maxDest
 * @param value Value to remap
 * @param minOr Minium value in the original range
 * @param maxOr Maximum value in the original range
 * @param minDest Minimum value in the destination range
 * @param maxDest Maximum value in the destination range
 * @return Remapped value
 */
static double remap(const double& value, const double& minOr, const double& maxOr, const double& minDest, const double& maxDest) {
    if ( maxOr-minOr == 0 ) // Avoid division by zero
        return 0 ;
    else
        return ( (value-minOr)/(maxOr-minOr) ) * (maxDest-minDest) + minDest ;
}

#endif //EMODNET_QMGC_MISC_UTILS_H
