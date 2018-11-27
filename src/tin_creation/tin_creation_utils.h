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
 * TODO: Explain the setting of parameters (here and in the tutorial)
 *
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
