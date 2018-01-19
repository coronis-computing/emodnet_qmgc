//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_HIERARCY_POINT_SET_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_HIERARCY_POINT_SET_H

#include "tin_creator.h"
#include "cgal_defines.h"



class TinCreationSimplificationHierarchyPointSet : public TINCreationStrategy
{
public:
    TinCreationSimplificationHierarchyPointSet(double maxClusterSize,
                                               double maxSurfaceVariance,
                                               double borderSimplificationMaxDistance)
            : m_maxClusterSize(maxClusterSize)
            , m_maxSurfaceVariance(maxSurfaceVariance)
            , m_borderSimpMaxSqDist(borderSimplificationMaxDistance*borderSimplificationMaxDistance) {} ;

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices) ;

private:
    unsigned int m_maxClusterSize;
    double m_maxSurfaceVariance;
    double m_borderSimpMaxSqDist;

    void divideInputPoints(const PointCloud& pts, // Note: points in the borders to maintain will be removed from this list!
                           const bool &constrainEasternVertices,
                           const bool &constrainWesternVertices,
                           const bool &constrainNorthernVertices,
                           const bool &constrainSouthernVertices,
                           PointCloud& ptsToMaintain,
                           PointCloud& ptsToSimplify ) const;

    // Precondition: poly.normalize_border() has been executed, and is valid
    // NOTE: The <X>BorderVertices variables include the corner points!
    void extractTileVerticesAtBorders(const Polyhedron& poly,
                                      PointCloud& easternBorderVertices,
                                      PointCloud& westernBorderVertices,
                                      PointCloud& northernBorderVertices,
                                      PointCloud& southernBorderVertices,
                                      Point_3& cornerVertex00,
                                      Point_3& cornerVertex01,
                                      Point_3& cornerVertex10,
                                      Point_3& cornerVertex11) const;
    void getAllNonBorderVertices(const Polyhedron& poly, PointCloud& nonBorderPts) const;

    /// Simplify the north or south border points. Points are projected to the YZ plane and simplified there.
    PointCloud simplifyEastOrWestBorder(const PointCloud& pts) const ;

    /// Simplify the north or south border points. Points are projected to the XZ plane and simplified there.
    PointCloud simplifyNorthOrSouthBorder(const PointCloud& pts) const ;

    /// Simplify a set of polylines based on their elevation, while preserving the topology in the XY plane
    Polylines simplifyPolylines(const Polylines& polylines) const ;

};


#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_HIERARCY_POINT_SET_H
