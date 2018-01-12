//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#include "cgal_utils.h"

void delaunayToOFF( const std::string &outFilePath, const Delaunay &dt )
{
    std::ofstream ofs( outFilePath.c_str() );
    ofs << "OFF\n"  << dt.number_of_vertices()
    << " "  << dt.number_of_faces() << " 0" << std::endl;

    std::map<Delaunay::Vertex_handle,int> indices;
    int counter = 0;

    for(Delaunay::Finite_vertices_iterator it = dt.finite_vertices_begin(); it != dt.finite_vertices_end(); ++it)
    {
        ofs << it->point() << std::endl;
        indices.insert(std::pair<Delaunay::Vertex_handle,int>(it, counter++));
    }

    for(Delaunay::Finite_faces_iterator it = dt.finite_faces_begin(); it != dt.finite_faces_end(); ++it)
    {
        ofs << "3 " << indices[it->vertex(0)]
        << " "  << indices[it->vertex(1)]
        << " "  << indices[it->vertex(2)] << std::endl;
    }

    ofs.close() ;
}


bool isTileCorner( Polyhedron::Halfedge_const_handle e )
{
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



bool isBorder(Polyhedron::Vertex_handle& v)
{
    Polyhedron::Halfedge_around_vertex_const_circulator hv = v->vertex_begin();
    //move around the vertex and check if there is a halfedge which
    //is on border
    for(; hv != v->vertex_begin(); v++){
        if(hv->is_border())
            return true;
    }
    return false;
}


bool isBorder(Polyhedron::Vertex_const_handle& v)
{
    Polyhedron::Halfedge_around_vertex_const_circulator hv = v->vertex_begin();
    //move around the vertex and check if there is a halfedge which
    //is on border
    for(; hv != v->vertex_begin(); v++){
        if(hv->is_border())
            return true;
    }
    return false;
}



bool isPointInArc( const Point_2& query, const Point_2& center, const Point_2& p0, const Point_2& p1 )
{
    Vector_2 a = p0 - center ;
    Vector_2 b = query - center ;
    Vector_2 c = p1 - center ;

    double AxB = a.x()*b.y() - a.y()*b.x() ;
    double AxC = a.x()*c.y() - a.y()*c.x() ;
    double CxB = c.x()*b.y() - c.y()*b.x() ;
    double CxA = c.x()*a.y() - c.y()*a.x() ;

    return ( AxB*AxC >= 0. && CxB*CxA >= 0. ) ;
}


