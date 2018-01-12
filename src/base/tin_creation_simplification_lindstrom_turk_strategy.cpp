//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_lindstrom_turk_strategy.h"
#include "cgal_defines.h"
#include "cgal_border_edges_are_constrained_edge_map.h"
#include "cgal_corner_vertices_are_constrained_vertex_map.h"
#include "cgal_further_constrained_placement.h"
#include "cgal_cost_and_count_stop_predicate.h"



Polyhedron TINCreationSimplificationLindstromTurkStrategy::create( const std::vector<Point_3>& dataPts,
                                                           const bool& constrainEasternVertices,
                                                           const bool& constrainWesternVertices,
                                                           const bool& constrainNorthernVertices,
                                                           const bool& constrainSouthernVertices ) const
{
    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromDelaunay<Gt, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // Set up the edge constrainer
    typedef SMS::FurtherConstrainedPlacement<SimplificationPlacement,
    BorderEdgesAreConstrainedEdgeMap,
    CornerVerticesAreConstrainedVertexMap > SimplificationConstrainedPlacement ;
    BorderEdgesAreConstrainedEdgeMap wsbeac( surface,
                                             constrainEasternVertices,
                                             constrainWesternVertices,
                                             constrainNorthernVertices,
                                             constrainSouthernVertices ) ;
    CornerVerticesAreConstrainedVertexMap cvacvm(surface) ;
    SimplificationConstrainedPlacement scp( wsbeac, cvacvm ) ;
    SimplificationCost sc( SimplificationCostParams( m_weightVolume,
                                                     m_weightBoundary,
                                                     m_weightShape ) ) ;

    // TODO: Find a way to provide an intuitive stop predicate based on cost...
    //    SMS::Cost_and_count_stop_predicate<Polyhedron> cacsp(m_options.SimpStopCost,
    //                                                         m_options.SimpStopEdgesCount) ;

    //    typedef SMS::Constrained_placement<SimplificationPlacement, WesternAndSouthernBorderEdgesAreConstrainedEdgeMap > Placement;
    //    Placement pl(wsbeac) ;

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
                      .edge_is_constrained_map(wsbeac)
            ) ;

    return surface ;
}
