//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H

#include "tin_creation/tin_creator.h"
#include "cgal_defines.h"
#include "cgal/Projection_traits_3_extended.h"
#include "cgal/squared_distance_3_cost.h"


class TinCreationSimplificationPointSet : public TINCreationStrategy
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
    TinCreationSimplificationPointSet(double borderSimplificationMaxDistance)
            : m_borderSimpMaxSqDist(borderSimplificationMaxDistance*borderSimplificationMaxDistance) {} ;

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices) ;

    /**
     * While the process of preserving border/feature edges is implemented in this base class, the methods using point
     * set simplification differ on the way they simplify the point set
     * Thus, this is the only method that needs to be implemented in the child classes
     * @param pts Points to simplify
     * @return Simplified point set
     */
    virtual std::vector<Point_3> simplify(const std::vector<Point_3>& pts) = 0;

private:
    double m_borderSimpMaxSqDist;
    CTXY m_cdt;

    /// Imposes the required constraints to the internal CDT structure. Simplified the border/feature polylines when needed
    void imposeConstraints(Polyhedron& surface,
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

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
