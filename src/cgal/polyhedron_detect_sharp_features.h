//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_CGAL_POLYHEDRON_DETECT_SHARP_EDGES_H
#define EMODNET_TOOLS_CGAL_POLYHEDRON_DETECT_SHARP_EDGES_H

#include "tin_creation/tin_creation_cgal_types.h"
#include <CGAL/Mesh_3/Detect_features_in_polyhedra.h>

// We need the sharp edges to be detected independently of the border edges, which need to be treated differently
// We base this code on the detect_features function in the CGAL/Polyhedral_mesh_domain_with_features_3.h file

/**
 * @class Detect_sharp_features_in_polyhedra
 * @brief Extension of CGAL::Mesh_3::Detect_features_in_polyhedra<Polyhedron_> class, this time detecting sharp features
 * only, NOT INCLUDING borders
 */
template <typename Polyhedron_>
class Detect_sharp_features_in_polyhedra : public CGAL::Mesh_3::Detect_features_in_polyhedra<Polyhedron_> {
    typedef Polyhedron_ Polyhedron;
public:
    typedef typename Polyhedron::Traits Geom_traits;
    typedef typename Geom_traits::Vector_3 Vector_3;
    typedef typename Geom_traits::FT FT;

    typedef typename Polyhedron::Halfedge_handle Halfedge_handle;
    typedef typename Polyhedron::Facet_handle Facet_handle;
    typedef typename Polyhedron::Halfedge Halfedge;
    typedef typename Polyhedron::Facet Facet;
    typedef typename Facet::Patch_id Patch_id;
    typedef CGAL::Compare_handles_with_or_without_timestamps Compare_handles;

    typedef std::set <Facet_handle, Compare_handles> Facet_handle_set;
    typedef std::set <Halfedge_handle, Compare_handles> He_handle_set;

    void detect_sharp_features_only(Polyhedron& polyhedron,
                                 FT angle_in_deg = FT(60)) const;

// Need to reimplement the following because they are private methods... copied directly from the original source (CGAL 4.9)
private:
    Vector_3 facet_normal(const Facet_handle& f) const;
    bool is_sharp(const Halfedge_handle& he, FT cos_angle) const;
};

// Implementation of the member fuctions in Detect_sharp_features_in_polyhedra class
template <typename P_>
void
Detect_sharp_features_in_polyhedra<P_>::
detect_sharp_features_only(Polyhedron& polyhedron, FT angle_in_deg) const
{
    // Initialize vertices
    for(typename Polyhedron::Vertex_iterator v = polyhedron.vertices_begin(),
                end = polyhedron.vertices_end() ; v != end ; ++v)
    {
        v->nb_of_feature_edges = 0;
    }

    FT cos_angle ( std::cos(CGAL::to_double(angle_in_deg) * CGAL_PI / 180.) );

    // Detect sharp edges
    for(typename Polyhedron::Halfedge_iterator he = polyhedron.edges_begin(),
                end = polyhedron.edges_end() ; he != end ; ++he)
    {
        // The key change is the following line!
        if ( !he->is_border() && (angle_in_deg == FT() || is_sharp(he,cos_angle)) )
//        if(he->is_border() || angle_in_deg == FT() || is_sharp(he,cos_angle))
        {
            he->set_feature_edge(true);
            he->opposite()->set_feature_edge(true);

            ++he->vertex()->nb_of_feature_edges;
            ++he->opposite()->vertex()->nb_of_feature_edges;
        }
    }
}

template <typename P_>
typename Detect_sharp_features_in_polyhedra<P_>::Vector_3
Detect_sharp_features_in_polyhedra<P_>::
facet_normal(const Facet_handle& f) const
{
    Vector_3 sum = CGAL::NULL_VECTOR;
    typename Facet::Halfedge_around_facet_circulator h = f->facet_begin();

    do
    {
        Vector_3 normal = CGAL::cross_product(
                h->next()->vertex()->point() - h->vertex()->point(),
                h->next()->next()->vertex()->point() - h->next()->vertex()->point());

        FT sqnorm = normal * normal;
        if ( ! CGAL_NTS is_zero(sqnorm) )
        {
            normal = normal / CGAL::sqrt(sqnorm);
            sum = sum + normal;
        }
    }
    while (++h != f->facet_begin());

    FT sqnorm = sum * sum;

    return (! CGAL_NTS is_zero(sqnorm)) ? sum / CGAL::sqrt(sqnorm)
                                        : CGAL::NULL_VECTOR;
}


template <typename P_>
bool
Detect_sharp_features_in_polyhedra<P_>::
is_sharp(const Halfedge_handle& he, FT cos_angle) const
{
    Facet_handle f1 = he->facet();
    Facet_handle f2 = he->opposite()->facet();
    if(f1 == NULL || f2 == NULL)
        return false;

    const Vector_3& n1 = facet_normal(f1);
    const Vector_3& n2 = facet_normal(f2);

    if ( n1 * n2 <= cos_angle )
        return true;
    else
        return false;
}


/**
 * Detects sharp features in a Polyhedron
 * @tparam Polyhedron_
 * @param p
 * @param angle_in_deg
 */
template <typename Polyhedron_>
void detect_sharp_edges(Polyhedron_& p,
                        typename Polyhedron_::Traits::FT angle_in_deg)
{
    Detect_sharp_features_in_polyhedra<Polyhedron_> go;
    go.detect_sharp_features_only(p,angle_in_deg);
    go.detect_surface_patches(p);
    go.detect_vertices_incident_patches(p);
}

//
///**
// * @brief Modification of the detect_features function in the
// */
//
//template <typename Polyhedron_>
//std::vector<typename Polyhedron_::Point_3>
//detect_features_without_border(Polyhedron_& p, typename Polyhedron_::Traits::FT angleInDegree)
//{
//    // Get sharp features
//    detect_sharp_edges(p, angleInDegree);
//
////    // Get polylines
////    Polylines polys;
////
////    typedef std::back_insert_iterator<std::vector<Polyline> > Output_iterator;
////
//////    CGAL::Mesh_3::detect_polylines<Polyhedron,Polyline,Output_iterator>(
//////            &p, std::back_inserter(polylines));
//
//    // Get polylines using the built-in methods in Mesh_3 (we ignore contexts)
//    typedef std::vector<Point_3> Bare_polyline;
//    typedef CGAL::Mesh_3::Polyline_with_context<MeshDomain::Surface_patch_index, MeshDomain::Curve_segment_index,
//    Bare_polyline > Polyline;
//
//    std::vector<Polyline> polylines;
//    typedef std::back_insert_iterator<std::vector<Polyline> > Output_iterator;
//
//    CGAL::Mesh_3::detect_polylines<Polyhedron,Polyline,Output_iterator>(
//            &p, std::back_inserter(polylines));
//
//    // Extract the polylines without context information
//    CGAL::Mesh_3::Extract_bare_polyline<Polyline> extractor;
//
////    return polys;
//}

#endif //EMODNET_TOOLS_CGAL_POLYHEDRON_DETECT_SHARP_EDGES_H
