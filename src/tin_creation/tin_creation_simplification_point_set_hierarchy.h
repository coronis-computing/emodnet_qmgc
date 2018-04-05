//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

class TinCreationSimplificationPointSetHierarchy
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetHierarchy(double borderSimplificationMaxDistance,
                                               double borderSimplificationMaxLength,
                                               unsigned int minFeaturePolylineSize,
                                               unsigned int maxClusterSize,
                                               double maxSurfaceVariance)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance,
                                                borderSimplificationMaxLength,
                                                minFeaturePolylineSize)
    {
        m_maxClusterSizePerZoom = std::vector<unsigned int>{maxClusterSize};
        m_maxSurfaceVariancePerZoom = std::vector<double>{maxSurfaceVariance};
        setParamsForZoomConcreteStrategy(0);
    }

    TinCreationSimplificationPointSetHierarchy(const std::vector<double>& borderSimplificationMaxDistancePerZoom,
                                               const std::vector<double>& borderSimplificationMaxLengthPerZoom,
                                               unsigned int minFeaturePolylineSize,
                                               const std::vector<unsigned int>& maxClusterSizePerZoom,
                                               const std::vector<double>& maxSurfaceVariancePerZoom)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistancePerZoom,
                                                borderSimplificationMaxLengthPerZoom,
                                                minFeaturePolylineSize)
            , m_maxClusterSizePerZoom(maxClusterSizePerZoom)
            , m_maxSurfaceVariancePerZoom(maxSurfaceVariancePerZoom)
    {
        setParamsForZoomConcreteStrategy(0);
    }

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

    void setParamsForZoomConcreteStrategy(const unsigned int& zoom) {
        m_maxClusterSize = standardHandlingOfThresholdPerZoom(m_maxClusterSizePerZoom, zoom);
        m_maxSurfaceVariance = standardHandlingOfThresholdPerZoom(m_maxSurfaceVariancePerZoom, zoom);
    }

private:
    unsigned int m_maxClusterSize;
    double m_maxSurfaceVariance;
    std::vector<unsigned int> m_maxClusterSizePerZoom;
    std::vector<double> m_maxSurfaceVariancePerZoom;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H
