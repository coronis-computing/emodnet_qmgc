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

#ifndef EMODNET_QMGC_GENERATE_BORDER_FEATURES_POLYLINES_H
#define EMODNET_QMGC_GENERATE_BORDER_FEATURES_POLYLINES_H

#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal_utils.h"
#include <CGAL/centroid.h>

/**
 * @brief Gets the polylines corresponding to the edges on each of the 4 borders the tile if the corresponding flag
 * constrain<X>Border is set. If not set, the polyline is a line going from 0 to 1.
 *
 * @pre The normalize_border() function must have been called on @param surface before running this function
 * @param surface
 * @param constrainEastBorder
 * @param constrainWestBorder
 * @param constrainNorthBorder
 * @param constrainSouthBorder
 */
template <class Polyhedron>
std::vector<std::vector<typename Polyhedron::Point_3>>
generateBorderFeaturesPolylines( const Polyhedron& surface,
                                 const bool& constrainEastBorder = true,
                                 const bool& constrainWestBorder = true,
                                 const bool& constrainNorthBorder = true,
                                 const bool& constrainSouthBorder = true )
{
    typedef typename Polyhedron::Point_3                    Point_3;
    typedef typename Polyhedron::Traits::FT                 FT;
    typedef typename Polyhedron::Vertex_const_iterator      Vertex_const_iterator;
    typedef std::vector<Point_3>                            Polyline;
    typedef std::vector<Polyline>                           Polylines;

    Polylines polylines ;

    // Compute the middle point for reference
    std::vector<Point_3 > pts ;
    for ( Vertex_const_iterator it = surface.vertices_begin(); it != surface.vertices_end(); ++it )
        pts.push_back(it->point());
    Point_3 c3 = CGAL::centroid(pts.begin(), pts.end(),CGAL::Dimension_tag<0>());
    FT midX = c3.x() ;
    FT midY = c3.y() ;

    // NOTE: The naming of the functions is a bit misleading in CGAL's documentation...
    // The docs always refer to border halfedges as those halfedges incident to the "hole". However, the range [border_halfedges_begin(), halfedges_end()) includes ALL halfedges in the border, those who are incident to the hole AND ALSO their opposites, incident on a face.
    // For this reason, we check if this is a real "border halfedge", and take the opposite if it is not
    typename Polyhedron::Halfedge_const_handle startHE = surface.border_halfedges_begin() ;
    if (!startHE->is_border())
        startHE = startHE->opposite() ;
    typename Polyhedron::Halfedge_const_handle e = startHE ;

    Polyline plU; // Polyline for unconstrained border edges. This should be entered as a sequential polyline, not individual edges!
    bool prevIsRegularBorder = false ; // Checks wether the previously visited edge is a "regular" border edge, or it is a "constrained" border edge. When this is false, we should start a new polyline
    do {
        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        if ( ( constrainEastBorder && diffX < diffY && p0.x() > midX ) ||
             ( constrainWestBorder && diffX < diffY && p0.x() < midX ) ||
             ( constrainNorthBorder && diffY < diffX && p0.y() > midY ) ||
             ( constrainSouthBorder && diffY < diffX && p0.y() < midY ) ) {
            // We add this edge as a polyline, since polylines' endpoints are preserved by the meshing algorithm
            Polyline pl;
            pl.push_back(p0);
            pl.push_back(p1);

            polylines.push_back(pl);
            prevIsRegularBorder = false;
        }
        else {
            // The edge is a border edge, so we must constrain it to remain in the mesh, but we don't fix its vertices
            if (!prevIsRegularBorder) {
                if (!plU.empty())
                    polylines.push_back(plU);
                plU.clear() ;
                plU.push_back(p1) ;
                plU.push_back(p0) ;
                prevIsRegularBorder = true ;
            }
            else {
                plU.push_back(p0);
            }
        }

        if (prevIsRegularBorder)
            // If the edge is regular so far, check if it is incident to a corner (they need to be endpoints of the polylines to be maintained during meshing)
            prevIsRegularBorder = !isTileCorner<Polyhedron>(e) ;

        e = e->next() ; // Advancing in this way should circulate through the "hole", if we start in a border halfedge
    }
    while ( e != startHE ); // Finish when we reach the starting point

    if (!plU.empty())
        polylines.push_back(plU) ;

    return polylines ;
}

#endif //EMODNET_QMGC_GENERATE_BORDER_FEATURES_POLYLINES_H
