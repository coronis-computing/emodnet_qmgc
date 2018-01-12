/**
 * @file
 * @brief Modification of the example in CGAL (4.9) for simplifying a mesh maintaining border edges, located at:
 * CGAL/examples/Surface_mesh_simplification/edge_collapse_constrained_border_surface_mesh.cpp
 *
 * This version provides more user parameters to tune.
 *
 */

// STD
#include <iostream>
#include <fstream>
#include <map>
// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h> // Simplification function
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h> // Midpoint placement policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h> //Placement wrapper
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h> // Stop-condition policy based on a fixed number of desired output edges
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h> // // Stop-condition policy that stops when the number of undirected edges drops below a given % of the initial count
// Boost
#include <boost/program_options.hpp>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;

typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Surface_mesh>::edge_descriptor edge_descriptor;

namespace SMS = CGAL::Surface_mesh_simplification ;
using namespace std ;
namespace po = boost::program_options ;



//
// BGL property map which indicates whether an edge is marked as non-removable
//
struct Border_is_constrained_edge_map{
    const Surface_mesh* sm_ptr;
    typedef edge_descriptor key_type;
    typedef bool value_type;
    typedef value_type reference;
    typedef boost::readable_property_map_tag category;

    Border_is_constrained_edge_map(const Surface_mesh& sm)
    : sm_ptr(&sm)
    {}

    friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
        return  CGAL::is_border(edge, *m.sm_ptr);
    }
};



//
// Placement class
//
typedef SMS::Constrained_placement<SMS::Midpoint_placement<Surface_mesh>,
                                   Border_is_constrained_edge_map > Placement;



int main( int argc, char** argv )
{
    Surface_mesh surface_mesh;

    std::string inputFile, outputFile ;
    double simpRatio ;
    int numOutEdges ;
    bool constrainBorderEdges, constrainVertices ;
    po::options_description options("Simplifies a mesh using CGAL maintaining the edges on the border") ;
    options.add_options()
        ( "help,h", "Produce help message" )
        ( "input,i", po::value<std::string>(&inputFile), "Input mesh to simplify" )
        ( "output,o", po::value<std::string>(&outputFile)->default_value("out.off"), "The output OFF file with the simplified mesh" )
        ( "num-edges,e", po::value<int>(&numOutEdges)->default_value(1000), "Number of desired edges on the simplified mesh")
        // ( "out-percent,r", po::value<double>(&simpRatio)->default_value(0.5), "Simplification percentage")
        ( "constrain-border-edges", po::value<bool>(&constrainBorderEdges)->default_value(true), "Constrain border edges")
        ( "constrain-vertices", po::value<bool>(&constrainVertices)->default_value(false), "Constrain vertices")
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

    std::ifstream is(inputFile);
    if(!is){
        std::cerr<< "Filename provided is invalid\n";
        return 1;
    }

    cout << "- Reading input file..." ;
    is >> surface_mesh  ;
    cout << "done." << endl ;

    Surface_mesh::Property_map<halfedge_descriptor,std::pair<Point_3, Point_3> > constrained_halfedges;

    constrained_halfedges = surface_mesh.add_property_map<halfedge_descriptor,std::pair<Point_3, Point_3> >("h:vertices").first;

    std::size_t nb_border_edges=0;
    BOOST_FOREACH(halfedge_descriptor hd, halfedges(surface_mesh)){
        if(CGAL::is_border(hd,surface_mesh)){
            constrained_halfedges[hd] = std::make_pair(surface_mesh.point(source(hd,surface_mesh)),
                                                       surface_mesh.point(target(hd,surface_mesh)));
            ++nb_border_edges;
        }
    }

    // Stop predicate
    SMS::Count_stop_predicate<Surface_mesh> stop(numOutEdges);
//    SMS::Count_ratio_stop_predicate<Surface_mesh> stop(simpRatio) ;

    Border_is_constrained_edge_map bem(surface_mesh);

    cout << "- Simplifying " << flush ;
    // This the actual call to the simplification algorithm.
    // The surface mesh and stop conditions are mandatory arguments.
    int numRemoved = 0 ;
    if (constrainBorderEdges) {
        if (constrainVertices) {
            cout << "(Constraining vertices at border edges)..." << flush;
            numRemoved = SMS::edge_collapse
                    (surface_mesh, stop, CGAL::parameters::edge_is_constrained_map(bem)
                            .get_placement(Placement(bem))
                    );
        }
        else {
            cout << "(Constraining border edges)..." << flush;
            numRemoved = SMS::edge_collapse
                    (surface_mesh, stop, CGAL::parameters::edge_is_constrained_map(bem));
        }
    }
    else {
        cout << "..." << flush ;
        numRemoved = SMS::edge_collapse
                (surface_mesh, stop);
    }
    std::cout << "Finished...\n" << numRemoved << " edges removed.\n"
            << surface_mesh.number_of_edges() << " final edges.\n" ;

    cout << "- Saving output file..." << flush ;
    std::ofstream os( outputFile ) ;
        os << surface_mesh ;
    cout << "done." << endl ;

    // Check the
    if ( constrainVertices ) {
        BOOST_FOREACH(halfedge_descriptor hd, halfedges(surface_mesh)) {
            if (CGAL::is_border(hd, surface_mesh)) {
                --nb_border_edges;
                if (constrained_halfedges[hd] !=
                    std::make_pair(surface_mesh.point(source(hd, surface_mesh)),
                                   surface_mesh.point(target(hd, surface_mesh)))) {
                    std::cerr << "One of the edges was not preserved! :(\n";
                }
            }
        }
        assert( nb_border_edges==0 );
    }

    return 0 ;
}