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
 * @param eastBorderPolylines
 * @param westBorderPolylines
 * @param northBorderPolylines
 * @param southBorderPolylines
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
    while( e->is_border() )
    {
        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        Polyline pl;
        pl.push_back(p0);
        pl.push_back(p1);

        if ( constrainEastBorder && diffX < diffY && p0.x() > 0.5 )
            polylines.push_back(pl) ;
        else if ( constrainWestBorder && diffX < diffY && p0.x() < 0.5 )
            polylines.push_back(pl) ;
        else if ( constrainNorthBorder && diffY < diffX && p0.y() > 0.5 )
            polylines.push_back(pl) ;
        else if ( constrainSouthBorder && diffY < diffX && p0.y() < 0.5 )
            polylines.push_back(pl) ;

        std::advance(e,2) ;
    }

//    if (!constrainEastBorder) {
//        Point_3 ( 0.0, )
//        Polyline pl;
//        pl.push_back(p0);
//        pl.push_back(p1);
//    }

    return polylines ;
}

#endif //EMODNET_TOOLS_GENERATE_BORDER_FEATURES_POLYLINES_H
