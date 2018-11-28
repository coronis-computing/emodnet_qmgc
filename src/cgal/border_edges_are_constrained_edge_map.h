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

#ifndef EMODNET_QMGC_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
#define EMODNET_QMGC_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H

#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal_utils.h"


/**
 * @class BorderEdgesAreConstrainedEdgeMap
 *
 * @brief BGL property map indicating whether an edge is marked as non-removable
 *
 * We just need to mark as non-removable those edges corresponding to border edges of already processed neighboring tiles.
 */
template <class Polyhedron>
struct BorderEdgesAreConstrainedEdgeMap
{
    typedef typename boost::graph_traits<Polyhedron>::edge_descriptor key_type ;
    typedef bool value_type ;
    typedef value_type reference ;
    typedef boost::readable_property_map_tag category;
    typedef typename Polyhedron::Point_3 Point_3;

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

#endif //EMODNET_QMGC_CGAL_SIMPLIFICATION_CONSTRAINED_BORDERS_H
