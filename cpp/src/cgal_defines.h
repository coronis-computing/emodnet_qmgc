//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_CGAL_DEFINES_H
#define EMODNET_TOOLS_CGAL_DEFINES_H

/**
 * @brief Common types of CGAL used in the project
 */

// CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h> // Stop-condition policy based on a fixed number of desired output edges
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h> // // Stop-condition policy that stops when the number of undirected edges drops below a given % of the initial count
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>

// CGAL easy-to-use renaming of namespaces
namespace SMS = CGAL::Surface_mesh_simplification ;

// CGAL types
typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
typedef CGAL::Projection_traits_xy_3<K>                         Gt;
typedef CGAL::Delaunay_triangulation_2<Gt>                      Delaunay;
typedef K::Point_3                                              Point_3;
typedef K::Vector_3                                             Vector_3;

//typedef CGAL::Polyhedron_3<K,
//                           CGAL::Polyhedron_items_3,
//                           CGAL::HalfedgeDS_vector>             Polyhedron;
typedef CGAL::Polyhedron_3<K>                                   Polyhedron ;
typedef Polyhedron::HalfedgeDS                                  HalfedgeDS;
typedef Polyhedron::Halfedge_handle                             Halfedge_handle;
typedef Polyhedron::Vertex_handle                               Vertex_handle ;


typedef K::FT                                                   FT;
typedef CGAL::Min_sphere_of_points_d_traits_3<K,FT>             MinSphereTraits;
typedef CGAL::Min_sphere_of_spheres_d<MinSphereTraits>          MinSphere;
typedef MinSphereTraits::Sphere                                 Sphere;
typedef SMS::LindstromTurk_cost<Polyhedron>                     SimplificationCost ;
typedef SMS::Bounded_normal_change_placement<
             SMS::LindstromTurk_placement<Polyhedron> >         SimplificationPlacement ; // Note: Do not change this to use edge_length cost! While it lowers computational overhead, it will certainly destroy the border edges
typedef SMS::Count_ratio_stop_predicate<Polyhedron>             SimplificationStopPredicate ;

#endif //EMODNET_TOOLS_CGAL_DEFINES_H
