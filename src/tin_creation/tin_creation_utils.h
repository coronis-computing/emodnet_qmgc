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

#ifndef EMODNET_QMGC_TIN_CREATION_UTILS_H
#define EMODNET_QMGC_TIN_CREATION_UTILS_H

#include <vector>
#include <cmath>

namespace TinCreation {

/**
 * Some of the parameters are scale-dependant and, as such, they should depend on the zoom level of the pyramid.
 * If we refer to a parameter for a zoom z as p_z, we allow the user to just set p_z and then we compute the values for other zooms as p_z = p_0/2^z, for those parameters whose scale needs to be lowered at deeper levels, or p_z = p_0*2^z, for those whose scale needs to grow with depth.
 * The parameters marked with an asterisk (*) in the help of `qm_tiler` are using the setting proposed here.
 * Basically, depending on the number of values in the \p thresholdsPerZoom vector, we can:
 * * Set a single value: In this case, the value represents p_0, and the rest of values for the deeper zooms will be computed using the formulas presented in the previous paragraph.
 * * Set multiple values (by entering the same parameter more than once): In this way we can set a parameter for each zoom. If the user inputs less parameters than the number of required zoom levels, the last value is used for all the rest of deeper zooms to create.
 * @tparam T Numeric type of the parameter
 * @param thresholdsPerZoom Vector of thresholds per zoom. As explained above, it may contain a single parameter also.
 * @param zoom Current zoom level.
 * @param downScale Flag indicating whether the value of the parameter should be downScaled with zoom or, otherwise, upscaled
 * @return
 */
template <class T>
T standardHandlingOfThresholdPerZoom(const std::vector<T>& thresholdsPerZoom,
                                     const unsigned int& zoom,
                                     const bool& downScale = true)
{
    T thres;
    if (thresholdsPerZoom.size() == 1) {
        // This means that only the root tolerance was specified, we will infer the tolerance at the desired zoom level by dividing by two the root number for each level
        if (zoom == 0)
            thres = thresholdsPerZoom[0];
        else {
            if (downScale)
                thres = thresholdsPerZoom[0] / pow(2.0, zoom);
            else
                thres = thresholdsPerZoom[0] * pow(2.0, zoom);
        }
    } else if (zoom < thresholdsPerZoom.size()) {
        // Use the approximation tolerance corresponding to the zoom in the vector
        thres = thresholdsPerZoom[zoom];
    } else {
        // Use the approximation tolerance corresponding to the last zoom specified in the vector
        thres = thresholdsPerZoom.back();
    }

    return thres;
}


} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_UTILS_H
