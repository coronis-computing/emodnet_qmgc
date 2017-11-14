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
//    return ( AxB*AxC > 0. && CxB*CxA > 0. ) ;
}