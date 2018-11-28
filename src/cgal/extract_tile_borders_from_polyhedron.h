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

#ifndef EMODNET_QMGC_CGAL_EXTRACT_TILE_BORDERS_FROM_POLYHEDRON_H
#define EMODNET_QMGC_CGAL_EXTRACT_TILE_BORDERS_FROM_POLYHEDRON_H

#include "tin_creation/tin_creation_cgal_types.h"
#include <CGAL/centroid.h>



/**
 * @brief Extracts the NSEW borders from a tile stored in Polyhedron form.
 * \pre We assume the outer border of the polygon when projected to the XY plane to have a square (or at least a rectangular) shape.
 * \pre normalize_borders() has been called before this function and is still valid
 */
template<class Polyhedron>
bool extractTileBordersFromPolyhedron(const Polyhedron& poly,
                                      std::vector<typename Polyhedron::Point_3>& easternBorderPts,
                                      std::vector<typename Polyhedron::Point_3>& westernBorderPts,
                                      std::vector<typename Polyhedron::Point_3>& northernBorderPts,
                                      std::vector<typename Polyhedron::Point_3>& southernBorderPts,
                                      typename Polyhedron::Point_3& cornerPoint00,
                                      typename Polyhedron::Point_3& cornerPoint01,
                                      typename Polyhedron::Point_3& cornerPoint10,
                                      typename Polyhedron::Point_3& cornerPoint11)
{
    typedef typename Polyhedron::Point_3 Point_3;
    typedef typename Polyhedron::Traits::FT FT;
    typedef typename Polyhedron::Vertex_const_iterator Vertex_const_iterator;
    typedef typename Polyhedron::Halfedge_const_iterator Halfedge_const_iterator;

    easternBorderPts.clear();
    westernBorderPts.clear();
    northernBorderPts.clear();
    southernBorderPts.clear();

    int numCorners = 0 ;

    std::vector<Point_3 > pts ;
    for ( Vertex_const_iterator it = poly.vertices_begin(); it != poly.vertices_end(); ++it )
        pts.push_back(it->point());
    Point_3 c3 = CGAL::centroid(pts.begin(), pts.end(),CGAL::Dimension_tag<0>());
    FT midX = c3.x() ;
    FT midY = c3.y() ;

    Halfedge_const_iterator e = poly.border_halfedges_begin() ;
    ++e ; // We start at the second halfedge!
    while( e->is_border() )
    {
        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        // Differences between the points in the edge
        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        // Check if it is a corner point: the next vertex changes from vertical to horizontal or viceversa
        // If it is a corner point, we should add it twice to the corresponding border

        // Next edge on the border (since we are in a border halfedge, the next operator points to the next halfedge around the "hole"
        Point_3 p2 = e->next()->vertex()->point() ;

        double diffXNext = fabs( p2.x() - p0.x() ) ;
        double diffYNext = fabs( p2.y() - p0.y() ) ;
        bool isCorner = ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
                        ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;

        if ( isCorner ) {
            numCorners++ ;
            if ( p0.x() < midX && p0.y() < midY ) { // Corner (0, 0)
                cornerPoint00 = p0;
                westernBorderPts.push_back(p0);
                southernBorderPts.push_back(p0);
            }
            else if ( p0.x() < midX && p0.y() > midY ) { // Corner (0, 1)
                cornerPoint01 = p0;
                westernBorderPts.push_back(p0);
                northernBorderPts.push_back(p0);
            }
            else if ( p0.x() > midX && p0.y() > midY ) { // Corner (1, 1)
                cornerPoint11 = p0;
                easternBorderPts.push_back(p0);
                northernBorderPts.push_back(p0);
            }
            else { // p0.x() > 0.5 && p0.y() < 0.5 ) // Corner (1, 0)
                cornerPoint10 = p0;
                easternBorderPts.push_back(p0);
                southernBorderPts.push_back(p0);
            }
        }
        else {
            if (diffX < diffY) {
                // Vertical edge, can be a western or eastern edge
                if (p0.x() < midX) {
                    // Western border edge/vertex
                    westernBorderPts.push_back(p0);
                } else { // p0.x() >= 0.5
                    // Eastern border vertex
                    easternBorderPts.push_back(p0);
                }
            } else { // diffX >= diffY
                // Horizontal edge, can be a northern or southern edge
                if (p0.y() < midY) {
                    // Southern border edge/vertex
                    southernBorderPts.push_back(p0);
                } else { // p0.y() >= 0.5
                    // Northern border edge/vertex
                    northernBorderPts.push_back(p0);
                }
            }
        }

        // Advance 2 positions (i.e., skip non-border halfedges)
        std::advance(e,2) ;
    }

    return numCorners == 4 ;
}

#endif //EMODNET_QMGC_CGAL_EXTRACT_TILE_BORDERS_FROM_POLYHEDRON_H
