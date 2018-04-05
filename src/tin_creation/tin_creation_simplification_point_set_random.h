//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

class TinCreationSimplificationPointSetRandom
        : public TinCreationSimplificationPointSet {
public:
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

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
