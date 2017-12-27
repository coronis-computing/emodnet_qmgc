//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_GENERATE_BORDER_FEATURES_POLYLINES_H
#define EMODNET_TOOLS_GENERATE_BORDER_FEATURES_POLYLINES_H

#include "cgal_defines.h"

/**
 * Gets the polylines corresponding to the edges on each of the 4 borders the tile if the corresponding flag
 * constrain<X>Border is set. If not set, the polyline is a line going from 0 to 1.
 *
 * @pre The normalize_border() function must have been called on @param surface before running this function
 * @param surface
 * @param constrainEastBorder
 * @param constrainWestBorder
 * @param constrainNorthBorder
 * @param constrainSouthBorder
 */
Polylines generateBorderFeaturesPolylines( const Polyhedron& surface,
                                           const bool& constrainEastBorder = true,
                                           const bool& constrainWestBorder = true,
                                           const bool& constrainNorthBorder = true,
                                           const bool& constrainSouthBorder = true )
{
    Polylines polylines ;

    Polyhedron::Halfedge_const_iterator e = surface.border_halfedges_begin() ;
    ++e ; // We start at the second halfedge!
    Polyline plU; // Polyline for unconstrained border edges. This should be entered as a sequential polyline, not individual edges!
    bool prevIsRegularBorder = false ; // Checks wether the previously visited edge is a "regular" border edge, or it is a "constrained" border edge. When this is false, we should start a new polyline
    while( e->is_border() )
    {
        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        if ( ( constrainEastBorder && diffX < diffY && p0.x() > 0.5 ) ||
             ( constrainWestBorder && diffX < diffY && p0.x() < 0.5 ) ||
             ( constrainNorthBorder && diffY < diffX && p0.y() > 0.5 ) ||
             ( constrainSouthBorder && diffY < diffX && p0.y() < 0.5 ) ) {
            // We add this edge as a polyline, since polylines' endpoints are preserved by the meshing algorithm
            Polyline pl;
            pl.push_back(p0);
            pl.push_back(p1);

            polylines.push_back(pl);
            prevIsRegularBorder = false;
            std::cout << "Single polyline" << std::endl ;
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
                std::cout << "p1 = " << p1 << std::endl ;
                std::cout << "p0 = " << p0 << std::endl ;
            }
            else {
                plU.push_back(p0) ;
                std::cout << "p0 = " << p0 << std::endl ;
            }
        }

        std::advance(e,2) ;
    }
    if (!plU.empty())
        polylines.push_back(plU) ;

    return polylines ;
}

#endif //EMODNET_TOOLS_GENERATE_BORDER_FEATURES_POLYLINES_H
