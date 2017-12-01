//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "surface_simplifier_remeshing_strategy.h"
#include "cgal_generate_border_features_polylines.h"
#include <CGAL/make_mesh_3.h>
#include <iostream>



void SurfaceSimplificationRemeshingStrategy::simplify( Polyhedron& surface,
                                                       const bool& constrainEasternVertices,
                                                       const bool& constrainWesternVertices,
                                                       const bool& constrainNorthernVertices,
                                                       const bool& constrainSouthernVertices ) const
{
    using namespace CGAL::parameters ;

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron*> poly_ptrs_vector(1, &surface);

    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    MeshDomain domain(poly_ptrs_vector.begin(), poly_ptrs_vector.end());

    // Get the border polylines to maintain
    Polylines polylines ;
    polylines = generateBorderFeaturesPolylines( surface,
                                                 constrainEasternVertices,
                                                 constrainWesternVertices,
                                                 constrainNorthernVertices,
                                                 constrainSouthernVertices ) ;
    domain.add_features(polylines.begin(), polylines.end());

    // Mesh criteria
    MeshCriteria criteria( edge_size = m_edgeSize,
                           facet_angle = m_facetAngle,
                           facet_size = m_facetSize,
                           facet_distance = m_facetDistance ) ;

    std::cout << "Meshing criteria:" << std::endl ;
    std::cout << "    - edge_size = " << m_edgeSize << std::endl ;
    std::cout << "    - facet_angle = " << m_facetAngle << std::endl ;
    std::cout << "    - facet_size = " << m_facetSize << std::endl ;
    std::cout << "    - facet_distance = " << m_facetDistance << std::endl ;

    // Mesh generation
    std::cout << "Meshing..." << std::flush ;
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
    std::cout << "done." << std::endl ;

//    // Output the facets of the c3t3 to an OFF file. The facets will not be oriented.
//    std::cout << "Saving resulting mesh (facets will not be oriented)..." << std::flush ;
//    std::ofstream off_file(outputFile);
//    c3t3.output_boundary_to_off(off_file);
//    std::cout << "done." << std::endl ;
}