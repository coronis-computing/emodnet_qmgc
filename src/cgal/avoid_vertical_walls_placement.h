//
// Created by Ricard Campos on 3/18/19.
//

#ifndef EMODNET_QUANTIZED_MESH_GENERATOR_FOR_CESIUM_AVOID_VERTICAL_WALLS_PLACEMENT_H
#define EMODNET_QUANTIZED_MESH_GENERATOR_FOR_CESIUM_AVOID_VERTICAL_WALLS_PLACEMENT_H

#include <limits>
#include <boost/optional.hpp>

namespace CGAL {
namespace Surface_mesh_simplification {
    /**
     * Placement that checks if another placement would generate a vertical face (i.e., all vertices sharing the same X or Y coordinate) around the stars of the two vertices of an edge that is candidate for an edge collapse.
     * It then rejects this placement by returning boost::none.
     * @tparam Placement
     */

    template<class Placement>
    class AvoidVerticalWallsPlacement {
    public:
        typedef typename Placement::ECM ECM;
//        typedef typename Placement::FT FT;

        /// Constructor
        AvoidVerticalWallsPlacement(const Placement& placement = Placement(), double eps = std::numeric_limits<double>::epsilon())
                : m_placement(placement), m_eps(eps) {}

        template <typename Profile>
        boost::optional<typename Profile::Point>
        operator()( Profile const& aProfile) const
        {
            // Get the result of the base placement
            boost::optional<typename Profile::Point> op = m_placement(aProfile);

            // If some placement is computed, check its validity
            if(op){
                // triangles returns the triangles of the star of the vertices of the edge to collapse
                // First the two trianges incident to the edge, then the other triangles
                // The second vertex of each triangle is the vertex that gets placed
                const typename Profile::Triangle_vector& triangles = aProfile.triangles();
                if(triangles.size()>2){
                    typedef typename Profile::Point Point;
                    typedef typename Profile::Kernel Traits;
                    typedef typename Traits::Vector_3 Vector;
                    typedef typename Traits::FT FT;

                    // Vector to compare against to detect vertical walls
                    Vector vwZ(0.0, 0.0, 1.0);

                    typename Profile::VertexPointMap ppmap = aProfile.vertex_point_map();
                    typename Profile::Triangle_vector::const_iterator it = triangles.begin();
                    if(aProfile.left_face_exists()){
                        ++it;
                    }
                    if(aProfile.right_face_exists()){
                        ++it;
                    }
                    while(it!= triangles.end()){
                        const typename Profile::Triangle& t = *it;
                        Point p = get(ppmap,t.v0);
                        // Point q = get(ppmap,t.v1); // <-- The point that will be placed!
                        Point r = get(ppmap,t.v2);
                        Point q2 = *op; // <-- The candidate placement

                        // Edges of the newly generated triangle
                        Vector eq2p = Traits().construct_vector_3_object()(q2,p) ;
                        Vector eq2r = Traits().construct_vector_3_object()(q2,r) ;

                        // Normal of the triangle
                        Vector n = Traits().construct_cross_product_vector_3_object()(eq2p,eq2r);

                        if( fabs(Traits().compute_scalar_product_3_object()(n, vwZ)) < m_eps ){
                            return boost::optional<typename Profile::Point>();
                        }
                        ++it;
                    }
                }
            }
            return op;

        }

    private:
        Placement  m_placement ;
        double m_eps;

    }; // End class AvoidVerticalWallsPlacement

} // End namespace Surface_mesh_simplification
} // End namespace CGAL

#endif //EMODNET_QUANTIZED_MESH_GENERATOR_FOR_CESIUM_AVOID_VERTICAL_WALLS_PLACEMENT_H
