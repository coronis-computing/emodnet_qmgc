//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_POLYHEDRON_BUILDER_FROM_PROJECTED_TRIANGULATION_H
#define EMODNET_TOOLS_POLYHEDRON_BUILDER_FROM_PROJECTED_TRIANGULATION_H

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


#endif //EMODNET_TOOLS_POLYHEDRON_BUILDER_FROM_PROJECTED_TRIANGULATION_H
