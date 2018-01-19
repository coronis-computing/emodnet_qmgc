//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_delaunay_strategy.h"
#include "cgal_defines.h"
#include "cgal/cgal_utils.h"


Polyhedron TINCreationDelaunayStrategy::create(const std::vector<Point_3>& dataPts,
                                               const bool &constrainEasternVertices,
                                               const bool &constrainWesternVertices,
                                               const bool &constrainNorthernVertices,
                                               const bool &constrainSouthernVertices)
{
    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    return surface ;
}