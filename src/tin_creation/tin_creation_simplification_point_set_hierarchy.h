//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H

#include "tin_creation_simplification_point_set.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSetHierarchy
 * @brief Creates a TIN using the Hierarchical point set simplification algorithm
 *
 * It uses CGAL's implementation of the method described in: <br>
 * Mark Pauly, Markus Gross, and Leif P Kobbelt. Efficient simplification of point-sampled surfaces. In Proceedings of the conference on Visualization'02, pages 163–170. IEEE Computer Society, 2002.
 */
class TinCreationSimplificationPointSetHierarchy
        : public TinCreationSimplificationPointSet
{
public:
    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param maxClusterSize Maximum cluster size
     * @param maxSurfaceVariance Maximum cluster variation value
     */
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

    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param maxClusterSize Maximum cluster size per zoom
     * @param maxSurfaceVariance Maximum cluster variation value per zoom
     */
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
