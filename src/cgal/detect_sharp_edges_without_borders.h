//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#ifndef EMODNET_TOOLS_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H
#define EMODNET_TOOLS_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>


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

#endif //EMODNET_TOOLS_DETECT_SHARP_EDGES_WITHOUT_BORDERS_H
