/**
 * \class
 *
 * \brief BGL property map which indicates whether an edge is marked as non-removable
 *
 * For the case of Quantized Meshes, and by the way they are constructed, we just need to mark as non-removable those
 * vertices on the western and southern border of the tile, which corresponds to maintain the edges of previous tiles
 * which connect with the current one in these borders.
 *
 */

#ifndef EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
#define EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H

#include "cgal_defines.h"



struct CGALSimplificationConstrainedBorders{

    typedef boost::graph_traits<Polyhedron>::edge_descriptor key_type ;
    typedef bool value_type ;
    typedef value_type reference ;
    typedef boost::readable_property_map_tag category ;

    const Polyhedron* m_pPolyPtr ;
    const bool m_constrainWestBorder ;
    const bool m_constrainSouthBorder ;

    CGALSimplificationConstrainedBorders( const Polyhedron& sm,
                                          const bool& constrainWestBorder = true,
                                          const bool& constrainSouthBorder = true )
            : m_pPolyPtr(&sm),
              m_constrainWestBorder(constrainWestBorder),
              m_constrainSouthBorder(constrainSouthBorder)
    {}

    friend bool get(CGALSimplificationConstrainedBorders m, const key_type& edge)
    {
        bool isBorder = CGAL::is_border(edge, *m.m_pPolyPtr);
        if (isBorder) {
            Point_3 p0 = edge.halfedge()->vertex()->point() ;
            Point_3 p1 = edge.halfedge()->opposite()->vertex()->point() ;

            return ( m.m_constrainWestBorder && p0.x() < DBL_EPSILON ) ||
                   ( m.m_constrainWestBorder && p1.x() < DBL_EPSILON ) ||
                   ( m.m_constrainSouthBorder && p0.y() < DBL_EPSILON ) ||
                   ( m.m_constrainSouthBorder && p1.y() < DBL_EPSILON ) ;
        }
        else
            return false ;
    }

};

#endif //EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
