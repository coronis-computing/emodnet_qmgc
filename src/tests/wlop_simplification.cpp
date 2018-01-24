//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include <vector>
#include <fstream>
#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal/cgal_utils.h"
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
// Boost
#include <boost/program_options.hpp>

using namespace std ;
using namespace TinCreation ;
namespace po = boost::program_options ;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

int main(int argc, char*argv[])
{
    double percentToRetain, neighborRadius ;
    std::string inputFile, outputFile;
    po::options_description options("Simplifies a mesh using CGAL maintaining the edges on the border") ;
    options.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<std::string>(&inputFile), "Data points to create the mesh (they are supposed to be projectible to the XY plane, i.e., be an implicit function of the form f(x,y) = z).")
            ("output,o", po::value<std::string>(&outputFile)->default_value("out.off"), "The output OFF file with the simplified mesh.")
            ("percent-to-retain", po::value<double>(&percentToRetain)->default_value(10), "Percentage of points to retain")
            ("neighbor-radius", po::value<double>(&neighborRadius)->default_value(0.5), "Neighbors size (automatic guess if smaller than zero)")
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

    std::vector<Point_3> points;
    std::ifstream stream(inputFile);
    if (!stream ||
        !CGAL::read_xyz_points(stream, std::back_inserter(points)))
    {
        std::cerr << "Error: cannot read file " << inputFile << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Read " << points.size () << " point(s)" << std::endl;
    CGAL::Timer task_timer; task_timer.start();

    // Simplification of the point set using WLOP
    std::vector<Point_3> simpPts ;
    CGAL::wlop_simplify_and_regularize_point_set
            <Concurrency_tag>
            (points.begin(),
             points.end(),
             std::back_inserter(simpPts),
             percentToRetain,
             neighborRadius
            );
    std::size_t memory = CGAL::Memory_sizer().virtual_size();

    std::cout << simpPts.size () << " point(s) kept, computed in "
              << task_timer.time() << " seconds, "
              << (memory>>20) << " Mib allocated." << std::endl;

    std::cout << "Triangulating the input points" << std::endl ;

    // Delaunay triangulation
    Delaunay dt( simpPts.begin(), simpPts.end() );

    std::cout << "Resulting triangulation has:" << std::endl ;
    std::cout << "  - " << dt.number_of_vertices() << " vertices" << std::endl ;
    std::cout << "  - " << dt.number_of_faces() << " triangles" << std::endl ;

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // Save the results
    std::ofstream of(outputFile);
    of << surface;

    return EXIT_SUCCESS;
}
