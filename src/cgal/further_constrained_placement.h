//
// Author: Ricard Campos (ricardcd@gmail.com)
//

/**
 * \brief Extension of CGAL's Constrained_placement class
 *
 * Extension of CGAL's Constrained_placement class in the CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h file.
 * The placement of the vertex resulting from a contraction of an edge containing a constrained vertex is that constrained vertex.
 *
 */

#ifndef EMODNET_TOOLS_CGAL_FURTHER_CONSTRAINED_PLACEMENT_H
#define EMODNET_TOOLS_CGAL_FURTHER_CONSTRAINED_PLACEMENT_H

#include <CGAL/Surface_mesh_simplification/Detail/Common.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_profile.h>

//using namespace CGAL ;
//using namespace Surface_mesh_simplification ;
#include <iostream>



namespace CGAL {

    namespace Surface_mesh_simplification {

        template<class BasePlacement, class EdgeIsConstrainedMap, class VertexIsConstrainedMap>
        class FurtherConstrainedPlacement : public BasePlacement {
        public:

            EdgeIsConstrainedMap Edge_is_constrained_map;
            VertexIsConstrainedMap Vertex_is_constrained_map;

        public:
            FurtherConstrainedPlacement(
                    EdgeIsConstrainedMap edgesMap = EdgeIsConstrainedMap(),
                    VertexIsConstrainedMap vertMap = VertexIsConstrainedMap(),
                    BasePlacement base = BasePlacement())
                    : BasePlacement(base), Edge_is_constrained_map(edgesMap), Vertex_is_constrained_map(vertMap) {}

            template<typename Profile>
            optional<typename Profile::Point> operator()(Profile const &aProfile) const {
                typedef typename Profile::ECM ECM;
                typedef typename CGAL::Halfedge_around_target_iterator<ECM> in_edge_iterator;

                // Constrained vertices
                bool isConstrainedV0 = get(Vertex_is_constrained_map, aProfile.v0()) ;
                bool isConstrainedV1 = get(Vertex_is_constrained_map, aProfile.v1()) ;

                if ( isConstrainedV0 && isConstrainedV1 ) {
                    // Both vertices in the edge are constrained: the edge must remain as is, no placement possible
//                    std::cout << "Both vertices in the edge are constrained: the edge must remain as is, no placement possible" << std::endl ;
                    return boost::optional<typename Profile::Point>();
                }
                else if ( isConstrainedV0 ) {
                    // v0 is constrained, return it as the placement
//                    std::cout << "v0 is constrained, return it as the placement" << std::endl ;
                    typename Profile::Point p = get(aProfile.vertex_point_map(), aProfile.v0()) ;
//                    std::cout << "v0 = " << p.x() << ", " << p.y() << ", " << p.z() << std::endl ;
                    return get(aProfile.vertex_point_map(), aProfile.v0()) ;
                }
                else if ( isConstrainedV1 ) {
//                    // v1 is constrained, return it as the placement
//                    std::cout << "v1 is constrained, return it as the placement" << std::endl ;
                    typename Profile::Point p = get(aProfile.vertex_point_map(), aProfile.v1()) ;
//                    std::cout << "v1 = " << p.x() << ", " << p.y() << ", " << p.z() << std::endl ;
                    return get(aProfile.vertex_point_map(), aProfile.v1()) ;
                }

                // Constrained edges
                in_edge_iterator eb, ee;
                for (boost::tie(eb, ee) = halfedges_around_target(aProfile.v0(), aProfile.surface_mesh());
                     eb != ee; ++eb) {
                    if (get(Edge_is_constrained_map, edge(*eb, aProfile.surface_mesh())))
                        return get(aProfile.vertex_point_map(),
                                   aProfile.v0());
                }
                for (boost::tie(eb, ee) = halfedges_around_target(aProfile.v1(), aProfile.surface_mesh());
                     eb != ee; ++eb) {
                    if (get(Edge_is_constrained_map, edge(*eb, aProfile.surface_mesh())))
                        return get(aProfile.vertex_point_map(),
                                   aProfile.v1());
                }



                return static_cast<const BasePlacement *>(this)->operator()(aProfile);
            }
        };

    }
}
#endif //EMODNET_TOOLS_CGAL_FURTHER_CONSTRAINED_PLACEMENT_H
