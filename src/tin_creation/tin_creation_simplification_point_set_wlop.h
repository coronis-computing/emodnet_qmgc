//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSetWLOP
 * @brief Creates a TIN using the Weighted Locally Optimal Projection (WLOP) algorithm
 *
 * We use the implementation of WLOP on the CGAL libraries, which are based on the following paper: <br>
 * H. Huang, D. Li, H. Zhang, U. Ascher, and D. Cohen-Or. Consolidation of unorganized point clouds for surface reconstruction. ACM Transactions on Graphics, 28:176:1–176:78, 2009.
 */
class TinCreationSimplificationPointSetWLOP
        : public TinCreationSimplificationPointSet
{
public:
    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param retainPercentage Percentage of points to retain
     * @param radius Spherical neighborhood radius
     * @param iterNumber Number of iterations to solve the optimization problem
     */
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

    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     * @param retainPercentage Percentage of points to retain per zoom
     * @param radius Spherical neighborhood radius per zoom
     * @param iterNumber Number of iterations to solve the optimization problem per zoom
     */
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
