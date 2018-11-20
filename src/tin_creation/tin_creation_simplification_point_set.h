//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H

#include "tin_creator.h"
#include "tin_creation_cgal_types.h"
#include "cgal/Projection_traits_3_extended.h"
#include "cgal/squared_distance_3_cost.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSet
 * @brief Creates a TIN using a point set simplification algorithm
 *
 * This is an interphase class for point set simplification methods. In this class, the main processing required to use
 * point set simplification techniques in square tiles is defined, but the actual point set simplification method is defined
 * in child classes, one for each method.
 *
 * Thus, this class is responsible of running the common part of these methods, which
 * is basically to detect border and feature edges in the original data, create polylines from those, and simplify them
 * separately. Once the polylines have been simplified, the rest of point sets are simplified (using the method implemented
 * in each child class) and triangulated in the XY plane.
 */
class TinCreationSimplificationPointSet : public TinCreationStrategy
{
    // --- Typedefs ---
    typedef CGAL::Projection_traits_xy_3_extended<K>                         ProjTraitsXY;
    typedef PS::Vertex_base_2<ProjTraitsXY>                                  VbXY;
    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsXY>        FbXY;
    typedef CGAL::Triangulation_data_structure_2<VbXY, FbXY>                 TDSXY;
    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsXY,
            TDSXY, CGAL::Exact_predicates_tag>                               CDTXY;
    typedef CGAL::Constrained_triangulation_plus_2<CDTXY>                    CTXY;
    typedef PS::Squared_distance_3_cost                                      PSSqDist3Cost;

public:
    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     */
    TinCreationSimplificationPointSet(double borderSimplificationMaxDistance,
                                      double borderSimplificationMaxLengthPercent,
                                      unsigned int minFeaturePolylineSize)
            : m_minFeaturePolylineSize(minFeaturePolylineSize)
    {
        m_borderSimpMaxDistPerZoom = std::vector<FT>{borderSimplificationMaxDistance};
        m_borderSimpMaxLengthPercentPerZoom = std::vector<FT>{borderSimplificationMaxLengthPercent};
        setParamsForZoom(0);
    }

    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     */
    TinCreationSimplificationPointSet(const std::vector<double>& borderSimplificationMaxDistance,
                                      const std::vector<double>& borderSimplificationMaxLengthPercent,
                                      unsigned int minFeaturePolylineSize)
            : m_borderSimpMaxDistPerZoom(borderSimplificationMaxDistance)
            , m_borderSimpMaxLengthPercentPerZoom(borderSimplificationMaxLengthPercent)
    {
        setParamsForZoom(0);
    }

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices) ;

    /**
     * Simplifies a point set
     * @param pts Points to simplify
     * @return Simplified point set
     */
    // While the process of preserving border/feature edges is implemented in this base class, the methods using point
    // set simplification differ on the way they simplify the point set
    // Thus, this is the only method that needs to be implemented in the child classes
    virtual std::vector<Point_3> simplify(const std::vector<Point_3>& pts) = 0;

    void setParamsForZoom(const unsigned int& zoom) {
        m_borderSimpMaxDist = standardHandlingOfThresholdPerZoom(m_borderSimpMaxDistPerZoom, zoom);

        m_borderSimpMaxLengthPercent = standardHandlingOfThresholdPerZoom(m_borderSimpMaxLengthPercentPerZoom, zoom);
        m_borderSimpMaxLengthPercent /= 100.0; // Convert to the range [0..1]

        // Set further parameters that are exclusive of each point set simplification strategy using setParamsForZoomConcreteStrategy(zoom); in derived classes
    }

    /**
     * Same as setParamsForZoom, but triggered at each child class
     * @param zoom Current zoom level
     */
    virtual void setParamsForZoomConcreteStrategy(const unsigned int& zoom) = 0;

private:
    double m_borderSimpMaxDist;
    double m_borderSimpMaxScaledSqDist;
    double m_borderSimpMaxLengthPercent; // We do not scale this length, as it is relative to the XY plane
    std::vector<double> m_borderSimpMaxDistPerZoom;
    std::vector<double> m_borderSimpMaxLengthPercentPerZoom;
    unsigned int m_minFeaturePolylineSize;
    CTXY m_cdt;

    /// Imposes the required constraints to the internal CDT structure. Simplified the border/feature polylines when needed
    void imposeConstraintsAndSimplifyPolylines(Polyhedron& surface,
                                               const bool &constrainEasternVertices,
                                               const bool &constrainWesternVertices,
                                               const bool &constrainNorthernVertices,
                                               const bool &constrainSouthernVertices) ;

    // Precondition: poly.normalize_border() has been executed, and is valid
    // NOTE: The <X>BorderVertices variables include the corner points!
//    void extractTileVerticesAtBorders(const Polyhedron& poly,
//                                      PointCloud& easternBorderVertices,
//                                      PointCloud& westernBorderVertices,
//                                      PointCloud& northernBorderVertices,
//                                      PointCloud& southernBorderVertices,
//                                      Point_3& cornerVertex00,
//                                      Point_3& cornerVertex01,
//                                      Point_3& cornerVertex10,
//                                      Point_3& cornerVertex11) const;
    void getAllNonBorderVertices(const Polyhedron& poly, PointCloud& nonBorderPts) const ;

    /// Simplify the north or south border points. Points are projected to the YZ plane and simplified there.
    PointCloud simplifyEastOrWestBorder(const PointCloud& pts) const ;

    /// Simplify the north or south border points. Points are projected to the XZ plane and simplified there.
    PointCloud simplifyNorthOrSouthBorder(const PointCloud& pts) const ;

    /// Simplify a set of polylines based on their elevation, while preserving the topology in the XY plane
    Polylines simplifyPolylines(const Polylines& polylines) const ;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
