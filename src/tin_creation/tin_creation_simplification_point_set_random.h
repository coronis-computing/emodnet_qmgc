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

#ifndef EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
#define EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSetRandom
 * @brief Creates a TIN using a random simplification
 *
 * It basically selects randomly a given subset of the original points.
 */
class TinCreationSimplificationPointSetRandom
        : public TinCreationSimplificationPointSet {
public:

/**
 * Constructor
 * @param borderSimplificationMaxDistance Maximum error for polyline simplification
 * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
 * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
 * @param removePercentage Percentage of points to remove from the input set
 */
TinCreationSimplificationPointSetRandom(double borderSimplificationMaxDistance,
                                        double borderSimplificationMaxLength,
                                        unsigned int minFeaturePolylineSize,
                                        double removePercentage)
        : TinCreationSimplificationPointSet(borderSimplificationMaxDistance,
                                            borderSimplificationMaxLength,
                                            minFeaturePolylineSize)
{
    m_removePercentagePerZoom = std::vector<double>{removePercentage};
    setParamsForZoomConcreteStrategy(0);
}

/**
 * Constructor
 * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
 * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
 * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
 * @param removePercentage Percentage of points to remove from the input set per zoom
 */
TinCreationSimplificationPointSetRandom(std::vector<double> borderSimplificationMaxDistancePerZoom,
                                        std::vector<double> borderSimplificationMaxLengthPerZoom,
                                        unsigned int minFeaturePolylineSize,
                                        std::vector<double> removePercentagePerZoom)
        : TinCreationSimplificationPointSet(borderSimplificationMaxDistancePerZoom,
                                            borderSimplificationMaxLengthPerZoom,
                                            minFeaturePolylineSize)
        , m_removePercentagePerZoom(removePercentagePerZoom)
{
    setParamsForZoomConcreteStrategy(0);
}

std::vector<Point_3> simplify(const std::vector<Point_3> &pts);

void setParamsForZoomConcreteStrategy(const unsigned int& zoom) {
    m_removePercentage = standardHandlingOfThresholdPerZoom(m_removePercentagePerZoom, zoom);
}

private:
// --- Attributes ---
double m_removePercentage;
std::vector<double> m_removePercentagePerZoom;
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
