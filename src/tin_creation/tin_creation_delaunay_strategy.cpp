//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#include "tin_creation_delaunay_strategy.h"
#include "tin_creation_cgal_types.h"
#include "cgal/polyhedron_builder_from_projected_triangulation.h"

namespace TinCreation {

Polyhedron TinCreationDelaunayStrategy::create(const std::vector<Point_3> &dataPts,
                                               const bool &constrainEasternVertices,
                                               const bool &constrainWesternVertices,
                                               const bool &constrainNorthernVertices,
                                               const bool &constrainSouthernVertices) {
    // Delaunay triangulation
    Delaunay dt(dataPts.begin(), dataPts.end());

    // Translate to Polyhedron
    Polyhedron surface;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    return surface;
}

} // End namespace TinCreation