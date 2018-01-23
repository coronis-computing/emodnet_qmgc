//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_POLYHEDRAL_MESH_DOMAIN_WITH_FEATURES_3_EXTENDED_H
#define EMODNET_TOOLS_POLYHEDRAL_MESH_DOMAIN_WITH_FEATURES_3_EXTENDED_H

#include <CGAL/Mesh_3/config.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include "polyhedron_detect_sharp_features.h"

namespace CGAL {

/**
 * @class Polyhedral_mesh_domain_with_features_3_extended
 *
 *
 */
    template<class IGT_,
            class Polyhedron_ = typename Mesh_polyhedron_3<IGT_>::type,
            class TriangleAccessor=Triangle_accessor_3<Polyhedron_, IGT_>,
            class Use_patch_id_tag = Tag_true,
            class Use_exact_intersection_construction_tag = Tag_true>
    class Polyhedral_mesh_domain_with_features_3_extended
            : public Polyhedral_mesh_domain_with_features_3<IGT_, Polyhedron_, TriangleAccessor, Use_patch_id_tag, Use_exact_intersection_construction_tag>
    {
        typedef Polyhedral_mesh_domain_with_features_3<IGT_, Polyhedron_, TriangleAccessor, Use_patch_id_tag, Use_exact_intersection_construction_tag> Base;

        typedef Polyhedron_ Polyhedron;

    public:
        // Index types
        typedef typename Base::Index Index;
        typedef typename Base::Corner_index Corner_index;
        typedef typename Base::Curve_segment_index Curve_segment_index;
        typedef typename Base::Surface_patch_index Surface_patch_index;
        typedef typename Base::Subdomain_index Subdomain_index;

        // Backward compatibility
#ifndef CGAL_MESH_3_NO_DEPRECATED_SURFACE_INDEX
        typedef Surface_patch_index Surface_index;
#endif // CGAL_MESH_3_NO_DEPRECATED_SURFACE_INDEX

        typedef typename Base::R R;
        typedef typename Base::Point_3 Point_3;
        typedef typename Base::FT FT;

        typedef CGAL::Tag_true Has_features;

        // New typedefs
        typedef std::vector<Point_3> Polyline;
        typedef std::vector<Polyline> Polylines;

        template <typename InputPolyhedraPtrIterator>
        Polyhedral_mesh_domain_with_features_3_extended(InputPolyhedraPtrIterator begin,
                                                        InputPolyhedraPtrIterator end,
                                                        CGAL::Random* p_rng = NULL)
                : Base(begin, end, p_rng) {}

        Polylines extract_features_without_borders(FT angle_in_degree, Polyhedron&p) {
//            for (std::size_t i = 0; i < polys.size(); ++i) {
//                Polyhedron &p = polys[i];
                this->initialize_ts(p);

                // Get sharp features
//                Mesh_3::detect_features(p, angle_in_degree);
                detect_sharp_edges(p, angle_in_degree);

                // Get polylines
                typedef Mesh_3::Polyline_with_context<Surface_patch_index, Curve_segment_index,
                        Polyline> PolylineWithContext;

                std::vector<PolylineWithContext> polylinesWithContext;
                typedef std::back_insert_iterator<std::vector<PolylineWithContext> > Output_iterator;

                Mesh_3::detect_polylines<Polyhedron, PolylineWithContext, Output_iterator>(
                        &p, std::back_inserter(polylinesWithContext));

                // Insert polylines in domain
                Mesh_3::Extract_bare_polyline<PolylineWithContext> extractor;

                Polylines polylines;
                polylines.insert(polylines.end(),
                                 boost::make_transform_iterator(polylinesWithContext.begin(), extractor),
                                 boost::make_transform_iterator(polylinesWithContext.end(), extractor));

                return polylines;
//            }
        }
    };


} // End namespace CGAL

#endif //EMODNET_TOOLS_POLYHEDRAL_MESH_DOMAIN_WITH_FEATURES_3_EXTENDED_H
