//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H

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
                                          double cellSize)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance,
                                                borderSimplificationMaxLength,
                                                minFeaturePolylineSize)
    {
        m_cellSizePerZoom = std::vector<double>{m_cellSize};
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
