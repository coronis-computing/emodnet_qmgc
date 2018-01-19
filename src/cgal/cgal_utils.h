//
// Created by Ricard Campos
//

#ifndef EMODNET_TOOLS_CGAL_UTILS_H
#define EMODNET_TOOLS_CGAL_UTILS_H

#include "cgal_defines.h"
#include <fstream>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_2.h>

/**
 * \brief A modifier creating a Polyhedron_3 structure with the incremental builder from a projected triangulation.
 * A "projected triangulation" is a Triangulation_2 with projection traits. That is, the triangulation was made on the plane, but the internal points are 3D.
 */
template<class ProjectedTriangulation2, class HDS>
class PolyhedronBuilderFromProjectedTriangulation : public CGAL::Modifier_base<HDS> {
public:
    typedef ProjectedTriangulation2 Tri;

    Tri m_dt ;

    PolyhedronBuilderFromProjectedTriangulation( const Tri &dt ) : m_dt(dt) {}

    void operator()( HDS& hds ) {
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;

        // Polyhedron_3 incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( m_dt.number_of_vertices(), m_dt.number_of_faces() );

        std::map<typename Tri::Vertex_handle,int> indices;
        int counter = 0 ;
        for(typename Tri::Finite_vertices_iterator it = m_dt.finite_vertices_begin();
            it != m_dt.finite_vertices_end(); ++it)
        {
            B.add_vertex( it->point() );
            indices.insert(std::pair<typename Tri::Vertex_handle,int>(it, counter++));
        }

        for(typename Tri::Finite_faces_iterator it = m_dt.finite_faces_begin();
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


/// Checks if a given halfedge is incident to a corner vertex in the tile
/// Note that this check only works on our specific case...
bool isTileCorner( Polyhedron::Halfedge_const_handle e ) ;



/// Exports a 2D Delaunay on a 3D point set (i.e., using the Projection_traits_xy_3) to an OFF file, for debugging purposes
void delaunayToOFF( const std::string &outFilePath, const Delaunay &dt ) ;

/// Check if a vertex in the polyhedron is in the border
bool isBorder(Polyhedron::Vertex_handle& v) ;
bool isBorder(Polyhedron::Vertex_const_handle& v) ;

/// Computes if a point \p p falls within the arc defined by the vectors  \p p0 - \p center and \p p1 - \p center
bool isPointInArc( const Point_2& query, const Point_2& center, const Point_2& p0, const Point_2& p1 ) ;

#endif //EMODNET_TOOLS_CGAL_UTILS_H
