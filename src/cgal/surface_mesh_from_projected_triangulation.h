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

#ifndef EMODNET_QMGC_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H
#define EMODNET_QMGC_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H

#include <map>

/**
 * @brief Creates a SurfaceMesh from a projected triangulation.
 * A "projected triangulation" is a Triangulation_2 with projection traits. That is, the triangulation was made on the plane, but the internal points are 3D.
 */
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

#endif //EMODNET_QMGC_SURFACE_MESH_FROM_PROJECTED_TRIANGULATION_H
