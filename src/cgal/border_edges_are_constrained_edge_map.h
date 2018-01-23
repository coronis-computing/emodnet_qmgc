#ifndef EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
#define EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H

#include "cgal_defines.h"
#include "cgal_utils.h"


/**
 * \class WesternAndSouthernBorderEdgesAreConstrainedEdgeMap
 *
 * \brief BGL property map indicating whether an edge is marked as non-removable
 *
 * For the case of Quantized Meshes, and by the way they are constructed, we just need to mark as non-removable those
 * edges corresponding to border edges of already processed neighboring tiles.
 *
 */
struct BorderEdgesAreConstrainedEdgeMap
{
    typedef boost::graph_traits<Polyhedron>::edge_descriptor key_type ;
    typedef bool value_type ;
    typedef value_type reference ;
    typedef boost::readable_property_map_tag category ;

    const Polyhedron* m_pPolyPtr ;
    const bool m_constrainEastBorder ;
    const bool m_constrainWestBorder ;
    const bool m_constrainNorthBorder ;
    const bool m_constrainSouthBorder ;

    BorderEdgesAreConstrainedEdgeMap( const Polyhedron& sm,
                                      const bool& constrainEastBorder = true,
                                      const bool& constrainWestBorder = true,
                                      const bool& constrainNorthBorder = true,
                                      const bool& constrainSouthBorder = true )
            : m_pPolyPtr(&sm)
            , m_constrainEastBorder(constrainEastBorder)
            , m_constrainWestBorder(constrainWestBorder)
            , m_constrainNorthBorder(constrainNorthBorder)
            , m_constrainSouthBorder(constrainSouthBorder) {}

    friend bool get(BorderEdgesAreConstrainedEdgeMap m, const key_type& edge)
    {
        bool isBorder = CGAL::is_border(edge, *m.m_pPolyPtr);
        if (isBorder) {
            Point_3 p0 = edge.halfedge()->vertex()->point() ;
            Point_3 p1 = edge.halfedge()->opposite()->vertex()->point() ;

            double diffX = fabs( p1.x() - p0.x() ) ;
            double diffY = fabs( p1.y() - p0.y() ) ;

            return ( m.m_constrainEastBorder && diffX < diffY && p0.x() > 0.5 ) ||
                   ( m.m_constrainWestBorder && diffX < diffY && p0.x() < 0.5 ) ||
                   ( m.m_constrainNorthBorder && diffY < diffX && p0.y() > 0.5 ) ||
                   ( m.m_constrainSouthBorder && diffY < diffX && p0.y() < 0.5 ) ;
        }
        else
            return false ;
    }
};

#endif //EMODNET_TOOLS_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
