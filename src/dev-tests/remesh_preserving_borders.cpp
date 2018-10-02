/**
 * @file
 * @brief Modification of the example in CGAL (4.9) for remeshing a polyhedron domain, located at:
 * CGAL/examples/Mesh_3/remesh_polyhedral_surface.cpp
 *
 * This version provides more user parameters to tune.
 *
 * @author Ricard Campos (rcampos@eia.udg.edu)
 */

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/make_mesh_3.h>
#include "cgal/polyhedron_builder_from_c3t3_boundary.h"

// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain;
// Polyhedron type
typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<
        Tr,Mesh_domain::Corner_index,Mesh_domain::Curve_segment_index> C3t3;
// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

typedef K::Point_3              Point_3;
typedef std::vector<Point_3>    Polyline;
typedef std::vector<Polyline>   Polylines;

// Boost
#include <boost/program_options.hpp>

using namespace CGAL::parameters; // To avoid verbose function and named parameters call
namespace po = boost::program_options ;
using namespace std ;



// Detect the border edges of the polyhedron, each border edge as a single polyline
Polylines myDetectBorders(Polyhedron& surface) {
    surface.normalize_border() ;

    Polylines polylines ;

    Polyhedron::Halfedge_iterator e = surface.border_halfedges_begin() ;
    ++e ; // We start at the second halfedge!
    while( e->is_border() )
    {
        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        // Single edge polyline
        Polyline pl;
        pl.push_back(p0);
        pl.push_back(p1);

        // Add to the list of polylines
        polylines.push_back(pl);

        std::advance(e,2) ;
    }

    return polylines ;
}



// Detect the border edges of the polyhedron, each border (set of connected border edges) as a single polyline
Polylines myDetectBordersSoft(Polyhedron& p) {
    Polylines polylines;
    Polyline empty;
    typedef typename boost::graph_traits<Polyhedron>::halfedge_descriptor halfedge_descriptor;

    std::set<halfedge_descriptor> visited;
    BOOST_FOREACH(halfedge_descriptor h, halfedges(p)) {
        if (visited.find(h) == visited.end()) {
            if (is_border(h, p)) {
                polylines.push_back(empty);
                Polyline &polyline = polylines.back();
                polyline.push_back(source(h, p)->point());
                BOOST_FOREACH(halfedge_descriptor h, halfedges_around_face(h, p)) {
                                polyline.push_back(target(h, p)->point());
                                visited.insert(h);
                            }
            } else {
                visited.insert(h);
            }
        }
    }

    return polylines ;
}


int main( int argc, char** argv )
{
    std::string inputFile, outputFile ;
    double simpRatio ;
    int numOutEdges ;
    bool constrainBorderEdges, constrainVertices, orientOutputSurface, keepSameBorderEdges ;
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
            ( "orient", po::value<bool>(&orientOutputSurface)->default_value(false), "Flag to try to force a coherent orientation for the output surface")
            ( "same_border_edges", po::value<bool>(&keepSameBorderEdges)->default_value(false), "Flag to force the original border edges to be fully maintained in the output surface")
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

    std::cout << "My CGAL library is " << CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl;

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

    std::cout << "poly.size_of_vertices() = " << poly.size_of_vertices() << std::endl ;
    std::cout << "poly.size_of_facets() = " << poly.size_of_facets() << std::endl ;

    if (poly.size_of_vertices() == 0 ) {
        std::cerr << "[ERROR] Either input geometry is empty or there has been a problem loading the file." << std::endl;
        return EXIT_FAILURE;
    }

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron*> poly_ptrs_vector(1, &poly);
    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    Mesh_domain domain(poly_ptrs_vector.begin(), poly_ptrs_vector.end());

    // Detect the border edges and mark them as features of the domain to preserve
//    poly.normalize_border() ;
    //domain.detect_borders();
//    domain.detect_features(); // Includes the detection of borders

    // My version
    Polylines polylines ;
    if (keepSameBorderEdges) {
        polylines = myDetectBorders(poly);
    }
    else {
        // Using our version just for testing purposes, CGAL has the same call in the domain.detect_borders() function
        polylines = myDetectBordersSoft(poly);
    }
    domain.add_features(polylines.begin(), polylines.end());

    // My other version
    Mesh_criteria criteria(edge_size = edgeSize,
                           facet_angle = facetAngle,
                           facet_size = facetSize,
                           facet_distance = facetDistance,
                           facet_topology = CGAL::MANIFOLD_WITH_BOUNDARY ); // Undocumented feature as of CGAL 4.9

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
    std::ofstream off_file(outputFile);
    if (orientOutputSurface) {
        Polyhedron surface;
        PolyhedronBuilderFromC3T3Boundary <C3t3, HalfedgeDS> builder(c3t3, 0);
        surface.delegate(builder);
        std::ofstream off_file(outputFile);
        off_file << surface ;
    }
    else {
        cout << "Saving resulting mesh (facets will not be oriented)..." << flush;
        c3t3.output_boundary_to_off(off_file);
        cout << "done." << endl;
    }

    return off_file.fail() ? EXIT_FAILURE : EXIT_SUCCESS;
}
