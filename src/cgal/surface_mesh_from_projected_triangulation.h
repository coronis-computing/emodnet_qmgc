//
// Author: Ricard Campos (ricardcd@gmail.com).
//

#ifndef EMODNET_TOOLS_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H
#define EMODNET_TOOLS_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H

#include <map>

template<class ProjectedTriangulation2, class SurfaceMesh>
SurfaceMesh surfaceMeshFromProjectedTriangulation(const ProjectedTriangulation2& tri)
{
    typedef ProjectedTriangulation2 Tri;
    typedef typename SurfaceMesh::vertex_index VertexIndex;
    SurfaceMesh sm;

    std::map<typename Tri::Vertex_handle, VertexIndex> indices;
    for(typename Tri::Finite_vertices_iterator it = tri.finite_vertices_begin();
        it != tri.finite_vertices_end(); ++it)
    {
        VertexIndex vi = sm.add_vertex(it->point());
        indices.insert(std::pair<typename Tri::Vertex_handle,int>(it, vi));
    }

    for(typename Tri::Finite_faces_iterator it = tri.finite_faces_begin();
        it != tri.finite_faces_end(); ++it)
    {
        sm.add_face(indices[it->vertex(0)], indices[it->vertex(1)], indices[it->vertex(2)]);
    }

    return sm;
}

#endif //EMODNET_TOOLS_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H
