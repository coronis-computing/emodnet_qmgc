//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

class TinCreationSimplificationPointSetWLOP
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetWLOP(double borderSimplificationMaxDistance,
                                          double borderSimplificationMaxLength,
                                          unsigned int minFeaturePolylineSize,
                                          double retainPercentage,
                                          double radius,
                                          unsigned int iterNumber = 35)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, borderSimplificationMaxLength, minFeaturePolylineSize)
            , m_iterNumber(iterNumber)
    {
        m_retainPercentagePerZoom = std::vector<double>{retainPercentage};
        m_radiusPerZoom = std::vector<double>{radius};
        setParamsForZoomConcreteStrategy(0);
    }

    TinCreationSimplificationPointSetWLOP(std::vector<double> borderSimplificationMaxDistancePerZoom,
                                          std::vector<double> borderSimplificationMaxLengthPerZoom,
                                          unsigned int minFeaturePolylineSize,
                                          std::vector<double> retainPercentagePerZoom,
                                          std::vector<double> radiusPerZoom,
                                          unsigned int iterNumber = 35)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistancePerZoom,
                                                borderSimplificationMaxLengthPerZoom,
                                                minFeaturePolylineSize)
            , m_retainPercentagePerZoom(retainPercentagePerZoom)
            , m_radiusPerZoom(radiusPerZoom)
            , m_iterNumber(iterNumber)
    {
        setParamsForZoomConcreteStrategy(0);
    }

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

    void setParamsForZoomConcreteStrategy(const unsigned int& zoom) {
        m_retainPercentage = standardHandlingOfThresholdPerZoom(m_retainPercentagePerZoom, zoom, false);
        m_radius = standardHandlingOfThresholdPerZoom(m_radiusPerZoom, zoom);
    }

private:
    // --- Attributes ---
    double m_retainPercentage;
    double m_radius;
    unsigned int m_iterNumber;
    std::vector<double> m_retainPercentagePerZoom; // percentage of points to retain.
    std::vector<double> m_radiusPerZoom; // neighbors size.
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H
