//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_CORNER_VERTICES_ARE_CONSTRAINED_VERTEX_MAP_H
#define EMODNET_TOOLS_CORNER_VERTICES_ARE_CONSTRAINED_VERTEX_MAP_H

#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal_utils.h"
#include <iostream>


/**
 * \class
 *
 * \brief BGL property map indicating whether a given vertex is one of the 4 corners of the tile.
 *
 * We need to preserve these corners when simplifying a tile.
 */
template <class Polyhedron>
struct CornerVerticesAreConstrainedVertexMap
{
    typedef typename boost::graph_traits<Polyhedron>::vertex_descriptor key_type;
    typedef typename boost::graph_traits<Polyhedron>::halfedge_descriptor  HalfedgeDescriptor;
    typedef bool value_type;
    typedef value_type reference;
    typedef typename Polyhedron::Point_3 Point_3;

    const Polyhedron *m_pPolyPtr;

    CornerVerticesAreConstrainedVertexMap( const Polyhedron &sm ) : m_pPolyPtr(&sm) {}

    friend bool get(CornerVerticesAreConstrainedVertexMap m, const key_type &vertex)
    {
        // The vertex must be on the border
        boost::optional<HalfedgeDescriptor> he = CGAL::is_border( vertex, *m.m_pPolyPtr ) ;

        if (he) {
            // Next edge on the border (since we are in a border halfedge, the next operator should point to the next halfedge around the "hole")
            if ( (*he)->next()->is_border() ) {
                // Check the differences in X and Y to discern if this is a corner vertex

                // Relevant geometric info of the current edge
                Point_3 p0 = vertex->point() ;
                Point_3 p1 = (*he)->opposite()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

                // Differences between the points in the edge
                double diffX = fabs( p1.x() - p0.x() ) ;
                double diffY = fabs( p1.y() - p0.y() ) ;

                Point_3 p2 = (*he)->next()->vertex()->point() ;

                double diffXNext = fabs( p2.x() - p0.x() ) ;
                double diffYNext = fabs( p2.y() - p0.y() ) ;

                return ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
                       ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;
            }
            else {
                return false ;
            }
        }
        else {
            return false ;
        }
    }
};

#endif // EMODNET_TOOLS_CORNER_VERTICES_ARE_CONSTRAINED_VERTEX_MAP_H
