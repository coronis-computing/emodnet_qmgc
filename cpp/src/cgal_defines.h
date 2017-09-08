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
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h> // Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>

// CGAL defines
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K> Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef K::Point_3 Point_3;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef K::FT FT;
typedef CGAL::Min_sphere_of_points_d_traits_3<K,FT> MinSphereTraits;
typedef CGAL::Min_sphere_of_spheres_d<MinSphereTraits> MinSphere;
typedef MinSphereTraits::Sphere                    Sphere;

// CGAL easy-to-use renaming of namespaces
namespace SMS = CGAL::Surface_mesh_simplification ;

#endif //EMODNET_TOOLS_CGAL_DEFINES_H
