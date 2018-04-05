//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

class TinCreationSimplificationPointSetGrid
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetGrid(double borderSimplificationMaxDistance,
                                          double borderSimplificationMaxLength,
                                          unsigned int minFeaturePolylineSize,
                                          double cellSize)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance,
                                                borderSimplificationMaxLength,
                                                minFeaturePolylineSize)
    {
        m_cellSizePerZoom = std::vector<double>{m_cellSize};
        setParamsForZoomConcreteStrategy(0);
    }

    TinCreationSimplificationPointSetGrid(const std::vector<double>& borderSimplificationMaxDistancePerZoom,
                                          const std::vector<double>& borderSimplificationMaxLengthPerZoom,
                                          unsigned int minFeaturePolylineSize,
                                          const std::vector<double>& cellSizePerZoom)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistancePerZoom,
                                                borderSimplificationMaxLengthPerZoom,
                                                minFeaturePolylineSize),
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

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
