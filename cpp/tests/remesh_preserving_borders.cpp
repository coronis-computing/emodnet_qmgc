/**
 * @file
 * @brief Modification of the example in CGAL (4.9) for remeshing a polyhedron domain, located at:
 * CGAL/examples/Mesh_3/remesh_polyhedral_surface.cpp
 *
 * This version provides more user parameters to tune.
 *
 */

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/make_mesh_3.h>

// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain;
// Polyhedron type
typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;
// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<
        Tr,Mesh_domain::Corner_index,Mesh_domain::Curve_segment_index> C3t3;
// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

// Boost
#include <boost/program_options.hpp>

using namespace CGAL::parameters; // To avoid verbose function and named parameters call
namespace po = boost::program_options ;
using namespace std ;


int main( int argc, char** argv )
{
    std::string inputFile, outputFile ;
    double simpRatio ;
    int numOutEdges ;
    bool constrainBorderEdges, constrainVertices ;
    double edgeSize, facetAngle, facetSize, facetDistance ;
    po::options_description options("Simplifies a mesh using CGAL maintaining the edges on the border") ;
    options.add_options()
            ( "help,h", "Produce help message" )
            ( "input,i", po::value<std::string>(&inputFile), "Input mesh to simplify" )
            ( "output,o", po::value<std::string>(&outputFile)->default_value("out.off"), "The output OFF file with the remeshed mesh" )
            ( "edge_size", po::value<double>(&edgeSize)->default_value(1), "Upper bound for the lengths of the boundary edges")
            ( "facet_angle", po::value<double>(&facetAngle)->default_value(25), "Lower bound for the angles (in degrees) of the surface mesh facets")
            ( "facet_size", po::value<double>(&facetSize)->default_value(0.1), "Upper bound for the radii of the surface Delaunay balls")
            ( "facet_distance", po::value<double>(&facetDistance)->default_value(0.001), "Upper bound for the distance between the facet circumcenter and the center of its surface Delaunay ball (i.e., approximation accuracy)")
    ;
    po::positional_options_description positionalOptions;
    positionalOptions.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(positionalOptions).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    // Load a polyhedron
    cout << "Loading the polyhedron from file..." << flush ;
    Polyhedron poly;
    std::ifstream input(inputFile);
    if(!input){
        std::cerr<< "Filename provided is invalid\n";
        return 1;
    }
    input >> poly;
    if (!CGAL::is_triangle_mesh(poly)){
        std::cerr << "Input geometry is not triangulated." << std::endl;
        return EXIT_FAILURE;
    }
    cout << "done." << endl ;

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron*> poly_ptrs_vector(1, &poly);
    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    Mesh_domain domain(poly_ptrs_vector.begin(), poly_ptrs_vector.end());

    // Detect the border edges and mark them as features of the domain to preserve
    domain.detect_borders();
    //domain.detect_features(); // Includes the detection of borders

    // Mesh criteria
    Mesh_criteria criteria(edge_size = edgeSize,
                           facet_angle = facetAngle,
                           facet_size = facetSize,
                           facet_distance = facetDistance);

    cout << "Meshing criteria:" << endl ;
    cout << "    - edge_size = " << edgeSize << endl ;
    cout << "    - facet_angle = " << facetAngle << endl ;
    cout << "    - facet_size = " << facetSize << endl ;
    cout << "    - facet_distance = " << facetDistance << endl ;

    // Mesh generation
    cout << "Meshing..." << flush ;
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
    cout << "done." << endl ;

    // Output the facets of the c3t3 to an OFF file. The facets will not be oriented.
    cout << "Saving resulting mesh (facets will not be oriented)..." << flush ;
    std::ofstream off_file(outputFile);
    c3t3.output_boundary_to_off(off_file);
    cout << "done." << endl ;

    return off_file.fail() ? EXIT_FAILURE : EXIT_SUCCESS;
}
