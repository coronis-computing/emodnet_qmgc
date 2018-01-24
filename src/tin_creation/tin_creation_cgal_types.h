//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_CGAL_DEFINES_H
#define EMODNET_TOOLS_CGAL_DEFINES_H

/**
 * @brief Common types of CGAL used in the project.
 *
 * We decided to fix the types because we don't envision to change the Kernel, and also because doing so we avoid the
 * use of templated classes.
 */

// CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// Remeshing (add these before, otherwise we get problems with some defines...)
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>

#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>

// Simplification
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h> // Stop-condition policy based on a fixed number of desired output edges
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h> // // Stop-condition policy that stops when the number of undirected edges drops below a given % of the initial count
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_params.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Stop_above_cost_threshold.h>
#include <CGAL/Polyline_simplification_2/Stop_below_count_ratio_threshold.h>
#include <CGAL/Polyline_simplification_2/Scaled_squared_distance_cost.h>

// STD
#include <vector>

namespace TinCreation {

// Renaming of namespaces
namespace SMS = CGAL::Surface_mesh_simplification;
namespace PS = CGAL::Polyline_simplification_2;

// CGAL types
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//typedef CGAL::Simple_cartesian<double>                          K;
typedef K::FT FT;
typedef CGAL::Projection_traits_xy_3<K> Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef K::Point_3 Point_3;
typedef K::Point_2 Point_2;
typedef K::Vector_2 Vector_2;
typedef K::Vector_3 Vector_3;
typedef K::Segment_2 Segment_2;
typedef std::vector<Point_3> Polyline;
typedef std::vector<Point_3> PointCloud;
typedef std::vector<Polyline> Polylines;

// Remeshing-related
typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> MeshDomain;
typedef CGAL::Mesh_triangulation_3<MeshDomain>::type Tr; // Triangulation
typedef CGAL::Mesh_complex_3_in_triangulation_3<
        Tr,
        MeshDomain::Corner_index,
        MeshDomain::Curve_segment_index> C3T3;
typedef CGAL::Mesh_criteria_3<Tr> MeshCriteria; // Criteria

//typedef CGAL::Polyhedron_3<K>                                   Polyhedron ;
typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef Polyhedron::Halfedge_handle Halfedge_handle;
typedef Polyhedron::Vertex_handle Vertex_handle;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<Polyhedron>::face_descriptor FaceDescriptor;
// The BGL makes use of indices associated to the vertices
// We use a std::map to store the index
typedef std::map<VertexDescriptor, Vector_3> VertexNormalMap;
// A std::map is not a property map, because it is not lightweight
typedef boost::associative_property_map<VertexNormalMap> VertexNormalPropertyMap;

// Simplification-related
typedef CGAL::Min_sphere_of_points_d_traits_3<K, FT> MinSphereTraits;
typedef CGAL::Min_sphere_of_spheres_d<MinSphereTraits> MinSphere;
typedef MinSphereTraits::Sphere Sphere;
typedef SMS::LindstromTurk_cost<Polyhedron> SimplificationCost;
typedef SMS::LindstromTurk_params SimplificationCostParams;
typedef SMS::Bounded_normal_change_placement<
        SMS::LindstromTurk_placement<Polyhedron> > SimplificationPlacement; // Note: Do not change this to use edge_length cost! While it lowers computational overhead, it will certainly destroy the border edges
typedef SMS::Count_stop_predicate<Polyhedron> SimplificationStopPredicate;

// Polyline simplification-related
//typedef CGAL::Polygon_2<Gt>                                     Polygon_2;
typedef PS::Stop_above_cost_threshold PSStopCost;
typedef PS::Stop_below_count_ratio_threshold PSStopCountRatio;
typedef PS::Squared_distance_cost PSSqDistCost;
typedef PS::Scaled_squared_distance_cost PSScaledSqDistCost;

#ifdef CGAL_LINKED_WITH_TBB
    typedef CGAL::Parallel_tag Concurrency_tag;
#else
    typedef CGAL::Sequential_tag Concurrency_tag;
#endif

} // End namespace tin_creation

#endif //EMODNET_TOOLS_CGAL_DEFINES_H
