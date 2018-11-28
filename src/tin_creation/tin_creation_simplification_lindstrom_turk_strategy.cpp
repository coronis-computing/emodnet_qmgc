// Copyright (c) 2018 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is part of EMODnet Quantized Mesh Generator for Cesium.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Author: Ricard Campos (ricardcd@gmail.com)

#include "tin_creation_simplification_lindstrom_turk_strategy.h"
#include "tin_creation_cgal_types.h"
#include "cgal/border_edges_are_constrained_edge_map.h"
#include "cgal/corner_vertices_are_constrained_vertex_map.h"
#include "cgal/further_constrained_placement.h"
#include "cgal/polyhedron_builder_from_projected_triangulation.h"

namespace TinCreation {

Polyhedron TinCreationSimplificationLindstromTurkStrategy::create( const std::vector<Point_3>& dataPts,
                                                                   const bool& constrainEasternVertices,
                                                                   const bool& constrainWesternVertices,
                                                                   const bool& constrainNorthernVertices,
                                                                   const bool& constrainSouthernVertices )
{
    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // Set up the edge constrainer
    typedef SMS::FurtherConstrainedPlacement<SimplificationPlacement,
                                             BorderEdgesAreConstrainedEdgeMap<Polyhedron>,
                                             CornerVerticesAreConstrainedVertexMap<Polyhedron> > SimplificationConstrainedPlacement ;
    BorderEdgesAreConstrainedEdgeMap<Polyhedron> beac( surface,
                                             constrainEasternVertices,
                                             constrainWesternVertices,
                                             constrainNorthernVertices,
                                             constrainSouthernVertices ) ;
    CornerVerticesAreConstrainedVertexMap<Polyhedron> cvacvm(surface) ;
    SimplificationConstrainedPlacement scp( beac, cvacvm ) ;
    SimplificationCost sc( SimplificationCostParams( m_weightVolume,
                                                     m_weightBoundary,
                                                     m_weightShape ) ) ;

    // TODO: Find a way to provide an intuitive stop predicate based on cost...
    int r = SMS::edge_collapse
            ( surface,
              SimplificationStopPredicate(m_stopEdgesCount),
//              cacsp,
              CGAL::parameters::vertex_index_map( get( CGAL::vertex_external_index,surface ) )
                      .halfedge_index_map(get(CGAL::halfedge_external_index, surface))
                      .get_cost(sc)
                              //                      .get_placement(pl)
                              //                      .get_placement(SimplificationPlacement())
                      .get_placement(scp)
                      .edge_is_constrained_map(beac)
            ) ;

    return surface ;
}


} // End namespace TinCreation