//
// Created by Ricard Campos
//

#ifndef EMODNET_TOOLS_CGAL_UTILS_H
#define EMODNET_TOOLS_CGAL_UTILS_H

#include "cgal_defines.h"
#include <fstream>
#include<CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/Polyhedron_3.h>
#include <CGAL/Delaunay_triangulation_2.h>

// A modifier creating a Polyhedron_3 structure with the incremental builder from a Delaunay triangulation
template<class Gt, class HDS>
class PolyhedronBuilder : public CGAL::Modifier_base<HDS> {
public:
    typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;

    Delaunay m_dt ;

    PolyhedronBuilder( const Delaunay &dt ) : m_dt(dt) {}

    void operator()( HDS& hds ) {
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;

        // Polyhedron_3 incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( m_dt.number_of_vertices(), m_dt.number_of_faces() );

        std::map<typename Delaunay::Vertex_handle,int> indices;
        int counter = 0 ;
        for(typename Delaunay::Finite_vertices_iterator it = m_dt.finite_vertices_begin();
            it != m_dt.finite_vertices_end(); ++it)
        {
            B.add_vertex( it->point() );
            indices.insert(std::pair<typename Delaunay::Vertex_handle,int>(it, counter++));
        }

        for(typename Delaunay::Finite_faces_iterator it = m_dt.finite_faces_begin();
            it != m_dt.finite_faces_end(); ++it)
        {
            B.begin_facet();
            B.add_vertex_to_facet( indices[it->vertex(0)] );
            B.add_vertex_to_facet( indices[it->vertex(1)] );
            B.add_vertex_to_facet( indices[it->vertex(2)] );
            B.end_facet();
        }

        // End the surface
        B.end_surface();
    }
};



// Exports a 2D Delaunay on a 3D point set (i.e., using the Projection_traits_xy_3) to an OFF file, for debugging purposes
void delaunayToOFF( const std::string &outFilePath, const Delaunay &dt ) ;

#endif //EMODNET_TOOLS_CGAL_UTILS_H
