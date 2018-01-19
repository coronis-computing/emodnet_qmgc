//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_remeshing_strategy.h"
#include "cgal/cgal_generate_border_features_polylines.h"
#include <CGAL/make_mesh_3.h>
#include <iostream>
#include "cgal/cgal_polyhedron_builder_from_c3t3_boundary.h"
#include <CGAL/config.h>
#include "cgal_defines.h"
#include <limits>



Polyhedron TINCreationRemeshingStrategy::create( const std::vector<Point_3>& dataPts,
                                                 const bool& constrainEasternVertices,
                                                 const bool& constrainWesternVertices,
                                                 const bool& constrainNorthernVertices,
                                                 const bool& constrainSouthernVertices )
{
    using namespace CGAL::parameters ;

    // First of all, check if the input data points are planar. If a planar mesh is input, the meshing algorithm never finishes!
    if (dataPtsArePlanar(dataPts)) {
        // Create a default triangulation for the grid and return
        std::vector<Point_3> defaultPts = defaultPointsForPlanarTile() ;

        // Delaunay triangulation
        Delaunay dt( defaultPts.begin(), defaultPts.end() );

        // Translate to Polyhedron
        Polyhedron surface ;
        PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
        surface.delegate(builderDT);

        return surface ;
    }

    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
    surface.delegate(builderDT);

    surface.normalize_border() ; // Needed to detect the borders

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron*> polyPtrsVector(1, &surface);

//    std::cout << "Creating the meshing domain" << std::endl ;

    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    MeshDomain domain(polyPtrsVector.begin(), polyPtrsVector.end());

//    std::cout << "Detecting the border lines" << std::endl ;

    // Get the border polylines to maintain
    Polylines polylines ;
    polylines = generateBorderFeaturesPolylines( surface,
                                                 constrainEasternVertices,
                                                 constrainWesternVertices,
                                                 constrainNorthernVertices,
                                                 constrainSouthernVertices ) ;
//    std::cout << "Adding features, imposing " << polylines.size() << " polylines" << std::endl ;
    domain.add_features(polylines.begin(), polylines.end());

//    for (int i = 0; i < polylines.size(); i++) {
//        std::cout << "Polyline" << i << " = [" << std::endl ;
//        for (int j = 0; j < polylines[i].size(); j++) {
//            std::cout << polylines[i][j] ;
//            if (j == polylines[i].size()-1)
//                std::cout << "]" << std::endl ;
//            else
//                std::cout << "; " ;
//        }
//    }

    // Mesh criteria
    // WARNING: Manifold criteria in mesh_3 is an undocumented feature as of CGAL 4.9.
    // In fact, the documented feature FACET_VERTICES_ON_SAME_SURFACE_PATCH_WITH_ADJACENCY_CHECK is not implemented!
    // See: https://github.com/CGAL/cgal/pull/590, and CGAL/Mesh_facet_topology.h
    MeshCriteria criteria(CGAL::parameters::edge_size = m_edgeSize,
                          CGAL::parameters::facet_angle = m_facetAngle,
                          CGAL::parameters::facet_size = m_facetSize,
                          CGAL::parameters::facet_distance = m_facetDistance,
                          CGAL::parameters::facet_topology = CGAL::MANIFOLD_WITH_BOUNDARY );

//    std::cout << "Meshing criteria:" << std::endl ;
//    std::cout << "    - edge_size = " << m_edgeSize << std::endl ;
//    std::cout << "    - facet_angle = " << m_facetAngle << std::endl ;
//    std::cout << "    - facet_size = " << m_facetSize << std::endl ;
//    std::cout << "    - facet_distance = " << m_facetDistance << std::endl ;
//    std::cout << "    - facet_topology = CGAL::MANIFOLD_WITH_BOUNDARY" << std::endl ;

    // Mesh generation
//    std::cout << "Meshing... " << std::flush ;
    C3T3 c3t3 = CGAL::make_mesh_3<C3T3>(domain, criteria, no_perturb(), no_exude());
//    std::cout << "done." << std::endl ;

    // Extract the surface boundary as a polyhedron
    Polyhedron remeshedSurface;
    PolyhedronBuilderFromC3T3Boundary<Gt, HalfedgeDS> builderC3T3Boundary(c3t3, 0);
    remeshedSurface.delegate(builderC3T3Boundary);

//    // Output the facets of the c3t3 to an OFF file. The facets will not be oriented.
////    std::cout << "Saving resulting mesh (facets will not be oriented)..." << std::flush ;
//    std::ofstream off_file("./remeshed.off");
////    c3t3.output_boundary_to_off(off_file);
//    off_file << remeshedSurface ;
////    std::cout << "done." << std::endl ;

    return remeshedSurface ;
}



bool TINCreationRemeshingStrategy::dataPtsArePlanar( const std::vector<Point_3>& dataPts ) const
{
    for ( std::vector<Point_3>::const_iterator it = dataPts.begin(); it != dataPts.end(); ++it ) {
        if (it->z() > std::numeric_limits<double>::epsilon())
            return false ;
    }

    return true ;
}



std::vector<Point_3> TINCreationRemeshingStrategy::defaultPointsForPlanarTile() const
{
    std::vector<Point_3> pts ;

    double step = 0.2 ;
    for ( double i = 0; i <= 1; i+=step ) {
        for ( double j = 0; j <= 1; j+=step ) {
            Point_3 p( i, j, 0 ) ;
            pts.push_back(p) ;
        }
    }

    return pts ;
}