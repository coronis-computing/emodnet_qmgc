//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_UTILS_H
#define EMODNET_TOOLS_TIN_CREATION_UTILS_H

#include <vector>
#include <cmath>

namespace TinCreation {

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

#endif //EMODNET_TOOLS_TIN_CREATION_UTILS_H
