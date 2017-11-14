//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

/**
 * \class
 *
 * \brief BGL property map indicating whether a vertex is marked as non-removable
 *
 * For the case of Quantized Meshes, and by the way they are constructed, we just need to mark as non-removable those
 * vertices on the western and southern border of the tile, which corresponds to maintain the edges of previous tiles
 * which connect with the current one in these borders.
 *
 */

#ifndef EMODNET_TOOLS_CORNERVERTEXISCONSTRAINEDMAP_H
#define EMODNET_TOOLS_CORNERVERTEXISCONSTRAINEDMAP_H

#include "cgal_defines.h"
#include "cgal_utils.h"
#include <iostream>



struct CornerVerticesAreConstrainedVertexMap
{
    typedef boost::graph_traits<Polyhedron>::vertex_descriptor key_type;
    typedef boost::graph_traits<Polyhedron>::halfedge_descriptor  HalfedgeDescriptor;
    typedef bool value_type;
    typedef value_type reference;

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

                bool isCorner = ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
                                ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;

//                if (isCorner) {
//                    std::cout << "p0 = " << p0 << std::endl ;
//                }

                return isCorner ;
//                return false ;
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

#endif //EMODNET_TOOLS_CORNERVERTEXISCONSTRAINEDMAP_H
