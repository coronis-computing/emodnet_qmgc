//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "surface_simplifier_remeshing_strategy.h"
#include "cgal_generate_border_features_polylines.h"
#include <CGAL/make_mesh_3.h>
#include <iostream>
#include "cgal_polyhedron_builder_from_c3t3_boundary.h"



void SurfaceSimplificationRemeshingStrategy::simplify( Polyhedron& surface,
                                                       const bool& constrainEasternVertices,
                                                       const bool& constrainWesternVertices,
                                                       const bool& constrainNorthernVertices,
                                                       const bool& constrainSouthernVertices ) const
{
    using namespace CGAL::parameters ;

    surface.normalize_border() ; // Needed to detect the borders

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron*> polyPtrsVector(1, &surface);

    std::cout << "Creating the meshing domain" << std::endl ;

    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    MeshDomain domain(polyPtrsVector.begin(), polyPtrsVector.end());

    std::cout << "Detecting the border lines" << std::endl ;

    // Get the border polylines to maintain
    Polylines polylines ;
    polylines = generateBorderFeaturesPolylines( surface,
                                                 constrainEasternVertices,
                                                 constrainWesternVertices,
                                                 constrainNorthernVertices,
                                                 constrainSouthernVertices ) ;
    std::cout << "Adding features, imposing " << polylines.size() << " polylines" << std::endl ;
    domain.add_features(polylines.begin(), polylines.end());

    // Mesh criteria
//    MeshCriteria criteria( edge_size = m_edgeSize,
//                           facet_angle = m_facetAngle,
//                           facet_size = m_facetSize,
//                           facet_distance = m_facetDistance ) ;

    MeshCriteria::Facet_criteria facet_criteria( m_facetAngle, // Facets' angle bound
                                                 m_facetSize,  // Facets' size bound
                                                 m_facetDistance, // Facets' distance bound
                                                 CGAL::FACET_VERTICES_ON_SAME_SURFACE_PATCH_WITH_ADJACENCY_CHECK); // Facets' adjacency check (to promote manifold surfaces)
    MeshCriteria::Cell_criteria cell_criteria( 2.,          // Cells' radius-edge ratio
                                               m_edgeSize); // Cells' size bound
    MeshCriteria criteria(facet_criteria, cell_criteria);

    std::cout << "Meshing criteria:" << std::endl ;
    std::cout << "    - edge_size = " << m_edgeSize << std::endl ;
    std::cout << "    - facet_angle = " << m_facetAngle << std::endl ;
    std::cout << "    - facet_size = " << m_facetSize << std::endl ;
    std::cout << "    - facet_distance = " << m_facetDistance << std::endl ;
    std::cout << "FACET_VERTICES_ON_SAME_SURFACE_PATCH_WITH_ADJACENCY_CHECK" << std::endl ;

    // Mesh generation
    std::cout << "Meshing..." << std::flush ;
    C3T3 c3t3 = CGAL::make_mesh_3<C3T3>(domain, criteria, no_perturb(), no_exude());
    std::cout << "done." << std::endl ;

    // Extract the surface boundary as a polyhedron
    Polyhedron surface2;
    PolyhedronBuilderFromC3T3Boundary<Gt, HalfedgeDS> builder(c3t3, 0);
    surface2.delegate(builder);

    surface = surface2 ;

//    // Output the facets of the c3t3 to an OFF file. The facets will not be oriented.
//    std::cout << "Saving resulting mesh (facets will not be oriented)..." << std::flush ;
    std::ofstream off_file("./remeshed.off");
    c3t3.output_boundary_to_off(off_file);
//    std::cout << "done." << std::endl ;
}