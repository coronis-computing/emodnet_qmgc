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

#ifndef EMODNET_QMGC_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H
#define EMODNET_QMGC_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

/**
 * @brief Detects sharp edges in a polygon without accounting for the borders (the original implementation in CGAL considers the borders as feature edges, and we do not want that).
 * @tparam PolygonMesh Mesh type
 * @tparam FT Float type
 * @tparam EdgeIsSharpMap EdgeIsSharpMap type
 * @tparam GT Geometric traits
 * @param pmesh Polygon mesh
 * @param angle_in_deg Angle threshold to consider an edge as sharp
 * @param edge_is_sharp_map The output map where the detected sharp edges will be stored
 */
template<typename PolygonMesh,
        typename FT,
        typename EdgeIsSharpMap,
        typename GT>
void detect_sharp_edges_without_borders(PolygonMesh &pmesh,
                                        FT angle_in_deg,
                                        EdgeIsSharpMap &edge_is_sharp_map) {
    FT cos_angle(std::cos(CGAL::to_double(angle_in_deg) * CGAL_PI / 180.));

    // Detect sharp edges
    BOOST_FOREACH(typename boost::graph_traits<PolygonMesh>::edge_descriptor ed, edges(pmesh)) {
        typename boost::graph_traits<PolygonMesh>::halfedge_descriptor he = halfedge(ed, pmesh);
        typedef typename boost::graph_traits<PolygonMesh>::face_descriptor face_descriptor;

        // We skip border edges
        if (is_border_edge(he, pmesh))
            continue;
        else {
            // Get the faces incident to this edge
            face_descriptor f1 = face(he, pmesh);
            face_descriptor f2 = face(opposite(he, pmesh), pmesh);

            // Compute their normal
            const typename GT::Vector_3 &n1 = CGAL::Polygon_mesh_processing::compute_face_normal(f1, pmesh);
            const typename GT::Vector_3 &n2 = CGAL::Polygon_mesh_processing::compute_face_normal(f2, pmesh);

            // Check the dihedral angle between them, and mark it as a sharp edge if it is larger than the threshold
            if (n1 * n2 <= cos_angle) {
                put(edge_is_sharp_map, edge(he, pmesh), true);
            }
        }
    }
}

#endif //EMODNET_QMGC_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H
