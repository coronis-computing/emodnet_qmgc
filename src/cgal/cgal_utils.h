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

#ifndef EMODNET_QMGC_CGAL_UTILS_H
#define EMODNET_QMGC_CGAL_UTILS_H

#include "tin_creation/tin_creation_cgal_types.h"
#include <fstream>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_2.h>

/**
 * @file
 * @brief Set of miscellaneous functions extending the functionality of the CGAL library
 */

/**
 * Checks if a given halfedge is incident to a corner vertex in the tile
 * Note that this check only works on our specific case...
 */
template<class Polyhedron>
bool isTileCorner( typename Polyhedron::Halfedge_const_handle e ) {
    typedef typename Polyhedron::Point_3 Point_3 ;

    if (e->is_border()) {
        Point_3 p0 = e->vertex()->point() ; // The vertex the halfedge is incident to
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        // Differences between the points in the edge
        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        // Next edge on the border (since we are in a border halfedge, the next operator points to the next halfedge around the "hole"
        Point_3 p2 = e->next()->vertex()->point() ;

        // Differences between the points in the next edge
        double diffXNext = fabs( p2.x() - p0.x() ) ;
        double diffYNext = fabs( p2.y() - p0.y() ) ;

        // Compare slopes to see if e is incident to a corner vertex (i.e., if p0 is a corner vertex)
        return ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
               ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;
    }
    else {
        return false ;
    }
}



/// Exports a 2D Delaunay on a 3D point set (i.e., using the Projection_traits_xy_3) to an OFF file, for debugging purposes
template <class Delaunay>
void delaunayToOFF( const std::string &outFilePath, const Delaunay &dt )
{
    std::ofstream ofs( outFilePath.c_str() );
    ofs << "OFF\n"  << dt.number_of_vertices()
        << " "  << dt.number_of_faces() << " 0" << std::endl;

    std::map<typename Delaunay::Vertex_handle,int> indices;
    int counter = 0;

    for(typename Delaunay::Finite_vertices_iterator it = dt.finite_vertices_begin(); it != dt.finite_vertices_end(); ++it)
    {
        ofs << it->point() << std::endl;
        indices.insert(std::pair<typename Delaunay::Vertex_handle,int>(it, counter++));
    }

    for(typename Delaunay::Finite_faces_iterator it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it)
    {
        ofs << "3 " << indices[it->vertex(0)]
            << " "  << indices[it->vertex(1)]
            << " "  << indices[it->vertex(2)] << std::endl;
    }

    ofs.close() ;
}



/// Checks if a vertex in the polyhedron is in the border
template <class Polyhedron>
bool isBorder(typename Polyhedron::Vertex_handle& v)
{
    typename Polyhedron::Halfedge_around_vertex_const_circulator hv = v->vertex_begin();
    //move around the vertex and check if there is a halfedge which
    //is on border
    do {
        if(hv->is_border())
            return true;
    }while (++hv != v->vertex_begin());
    return false;
}


/// Checks if a vertex in the polyhedron is in the border
template <class Polyhedron>
bool isBorder(typename Polyhedron::Vertex_const_handle& v)
{
    typename Polyhedron::Halfedge_around_vertex_const_circulator hv = v->vertex_begin();
    //move around the vertex and check if there is a halfedge which
    //is on border
    do {
        if(hv->is_border())
            return true;
    }while (++hv != v->vertex_begin());
    return false;
}



/// Computes if a point \p query falls within the arc defined by the vectors  \p p0 - \p center and \p p1 - \p center
template <class K>
bool isPointInArc( const typename K::Point_2& query,
                   const typename K::Point_2& center,
                   const typename K::Point_2& p0,
                   const typename K::Point_2& p1 )
{
    typedef typename K::Vector_2    Vector_2;

    Vector_2 a = p0 - center ;
    Vector_2 b = query - center ;
    Vector_2 c = p1 - center ;

    double AxB = a.x()*b.y() - a.y()*b.x() ;
    double AxC = a.x()*c.y() - a.y()*c.x() ;
    double CxB = c.x()*b.y() - c.y()*b.x() ;
    double CxA = c.x()*a.y() - c.y()*a.x() ;

    return ( AxB*AxC >= 0. && CxB*CxA >= 0. ) ;
}

#endif //EMODNET_QMGC_CGAL_UTILS_H
