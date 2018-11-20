//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H

#include "tin_creator.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationRemeshingStrategy
 * @brief Creates a TIN using Delaunay refinement algorithm
 *
 * In fact, the method used in this class is not a simplification but a remeshing process.
 *
 * Starting from a high resolution mesh, we obtain a new mesh following a set of requirements on the final mesh:
 *
 * - facet_distance: Upper bound for the distance between the facet circumcenter and the center of its surface Delaunay ball (i.e., approximation accuracy)
 * - facet_angle: Lower bound for the angles (in degrees) of the surface mesh facets
 * - facet_size: Upper bound for the radii of the surface Delaunay balls
 * - edge_size: Upper bound for the lengths of the boundary edges
 *
 * When the simplification takes place, all the coordinates of the vertices are normalized between 0..1
 * Note that this also means that, if the parameters are not set wisely, using this process we can
 * get a mesh of even larger complexity with respect to the original one!
 */
class TinCreationRemeshingStrategy : public TinCreationStrategy
{
public:
    TinCreationRemeshingStrategy( double facetDistance,
                                  double facetAngle,
                                  double facetSize,
                                  double edgeSize)
            : m_facetAngle(facetAngle)
    {
        m_facetDistancePerZoom = std::vector<double>{facetDistance};
        m_facetSizePerZoom = std::vector<double>{facetSize};
        m_edgeSizePerZoom = std::vector<double>{edgeSize};
        setParamsForZoom(0);
    }

    TinCreationRemeshingStrategy(const std::vector<double>& facetDistance,
                                 double facetAngle,
                                 const std::vector<double>& facetSize,
                                 const std::vector<double>& edgeSize)
            : m_facetDistancePerZoom(facetDistance)
            , m_facetAngle(facetAngle)
            , m_facetSizePerZoom(facetSize)
            , m_edgeSizePerZoom(edgeSize)
    {
        setParamsForZoom(0);
    }

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool& constrainEasternVertices,
                      const bool& constrainWesternVertices,
                      const bool& constrainNorthernVertices,
                      const bool& constrainSouthernVertices);

    // WARNING: The remeshing strategy should not be used for tiled rendering!
    void setParamsForZoom(const unsigned int& zoom) {
        m_facetDistance = standardHandlingOfThresholdPerZoom(m_facetDistancePerZoom, zoom);
        m_facetSize = standardHandlingOfThresholdPerZoom(m_facetSizePerZoom, zoom);
        m_edgeSize = standardHandlingOfThresholdPerZoom(m_edgeSizePerZoom, zoom);
    }

private:
    // Algorithm parameters
    double m_facetDistance;
    double m_facetAngle;
    double m_facetSize;
    double m_edgeSize;
    std::vector<double> m_facetDistancePerZoom;
    std::vector<double> m_facetSizePerZoom;
    std::vector<double> m_edgeSizePerZoom;

    // Internal functions

    /// Checks if all input points are collinear
    bool dataPtsArePlanar(const std::vector<Point_3>& dataPts) const;

    /// Sets the default points for a planar tile, using a set of planar points for refinement will not work (don't know why!)
    std::vector<Point_3> defaultPointsForPlanarTile() const;

    /// Splits a polyline into individual edges
    /// Pre-condition: poly contains a sorted set of points (either vertically or horizontally, depending on the border they come from)
    Polylines borderPolylineToIndividualEdges(Polyline& poly);
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H
