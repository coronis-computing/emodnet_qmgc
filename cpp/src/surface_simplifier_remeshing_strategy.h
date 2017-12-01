//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H


#include "surface_simplifier.h"

/**
 * Simplifies the surface using Delaunay refinement algorithm
 * In fact, it is not a simplification but a remeshing process. Starting from a high resolution mesh, we obtain a new
 * mesh following a set of requirements on the final mesh.
 * - facet_distance: Upper bound for the distance between the facet circumcenter and the center of its surface Delaunay ball (i.e., approximation accuracy)
 * - facet_angle: Lower bound for the angles (in degrees) of the surface mesh facets
 * - facet_size: Upper bound for the radii of the surface Delaunay balls
 * - edge_size: Upper bound for the lengths of the boundary edges
 * When the simplification takes place, all the coordinates of the vertices are normalized between 0..1
 * Note that this also means that, if the parameters are not set wisely, using this process we can
 * get a mesh of even larger complexity with respect to the original one!
 *
 */
class SurfaceSimplificationRemeshingStrategy : public SurfaceSimplificationStrategy
{
public:
    SurfaceSimplificationRemeshingStrategy( double facetDistance,
                                            double facetAngle,
                                            double facetSize,
                                            double edgeSize)
            : m_facetDistance(facetDistance)
            , m_facetAngle(facetAngle)
            , m_facetSize(facetSize)
            , m_edgeSize(edgeSize) {}

    void simplify( Polyhedron& surface,
                   const bool& constrainEasternVertices,
                   const bool& constrainWesternVertices,
                   const bool& constrainNorthernVertices,
                   const bool& constrainSouthernVertices ) const ;
private:
    // Algorithm parameters
    double m_facetDistance ;
    double m_facetAngle ;
    double m_facetSize ;
    double m_edgeSize ;

};

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFIER_REMESHING_STRATEGY_H
