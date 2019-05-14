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

#ifndef EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
#define EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSetGrid
 * @brief Creates a TIN using the Grid point set simplification algorithm
 *
 * A regular grid of points is overlayed on the data, and a representative point is selected from within each cell.
 */
class TinCreationSimplificationPointSetGrid
        : public TinCreationSimplificationPointSet
{
public:
    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param cellSize Cell size
     */
    TinCreationSimplificationPointSetGrid(double borderSimplificationMaxDistance,
                                          double borderSimplificationMaxLength,
                                          unsigned int minFeaturePolylineSize,
                                          bool preserveSharpEdges,
                                          double cellSize)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance,
                                                borderSimplificationMaxLength,
                                                minFeaturePolylineSize,
                                                preserveSharpEdges)
    {
        m_cellSizePerZoom = std::vector<double>{cellSize};
        setParamsForZoomConcreteStrategy(0);
    }

    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param cellSize Cell size
     */
    TinCreationSimplificationPointSetGrid(const std::vector<double>& borderSimplificationMaxDistancePerZoom,
                                          const std::vector<double>& borderSimplificationMaxLengthPerZoom,
                                          unsigned int minFeaturePolylineSize,
                                          bool preserveSharpEdges,
                                          const std::vector<double>& cellSizePerZoom)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistancePerZoom,
                                                borderSimplificationMaxLengthPerZoom,
                                                minFeaturePolylineSize,
                                                preserveSharpEdges),
              m_cellSizePerZoom(cellSizePerZoom)
    {
        setParamsForZoomConcreteStrategy(0);
    }

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

    void setParamsForZoomConcreteStrategy(const unsigned int& zoom) {
        m_cellSize = standardHandlingOfThresholdPerZoom(m_cellSizePerZoom, zoom);
    }

private:
    // --- Attributes ---
    double m_cellSize ;
    std::vector<double> m_cellSizePerZoom;
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
