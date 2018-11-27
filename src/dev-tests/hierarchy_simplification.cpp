// Copyright (c) 2018 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is part of EMODnet Quantized Mesh Generator for Cesium.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Author: Ricard Campos (ricardcd@gmail.com)

/**
 * @file
 * @brief Simplifies a mesh using CGAL maintaining the edges on the border
 * @author Ricard Campos (ricardcd@gmail.com)
 */

#include <vector>
#include <fstream>
#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
#include <CGAL/hierarchy_simplify_point_set.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Timer.h>
#include <CGAL/Memory_sizer.h>
#include <CGAL/IO/Polyhedron_iostream.h>
// Boost
#include <boost/program_options.hpp>

using namespace std ;
using namespace TinCreation ;
namespace po = boost::program_options ;



int main(int argc, char*argv[])
{
    int maxClusterSize ;
    double maxSurfaceVariance ;
    std::string inputFile, outputFile;
    po::options_description options("Simplifies a mesh using CGAL maintaining the edges on the border") ;
    options.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<std::string>(&inputFile), "Data points to create the mesh (they are supposed to be projectible to the XY plane, i.e., be an implicit function of the form f(x,y) = z).")
            ("output,o", po::value<std::string>(&outputFile)->default_value("out.off"), "The output OFF file with the simplified mesh.")
            ("max-cluster-size", po::value<int>(&maxClusterSize)->default_value(100), "Maximum cluster size.")
            ("max-surface-variance", po::value<double>(&maxSurfaceVariance)->default_value(0.01), "Max surface variation.")
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

    // simplification by clustering using erase-remove idiom
    points.erase (CGAL::hierarchy_simplify_point_set (points.begin (), points.end (),
                                                      maxClusterSize, // Max cluster size
                                                      maxSurfaceVariance), // Max surface variation
                  points.end ());
    std::size_t memory = CGAL::Memory_sizer().virtual_size();

    std::cout << points.size () << " point(s) kept, computed in "
              << task_timer.time() << " seconds, "
              << (memory>>20) << " Mib allocated." << std::endl;

    std::cout << "Triangulating the input points" << std::endl ;

    // Delaunay triangulation
    Delaunay dt( points.begin(), points.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // Save the results
    std::ofstream of(outputFile);
    of << surface;

    return EXIT_SUCCESS;
}
