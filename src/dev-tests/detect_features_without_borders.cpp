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
 * @brief Detects sharp edges in a surface mesh without including border edges (PMP::sharp_edges_segmentation includes them)
 *
 * Note that this example uses the Surface_mesh class (and its internal property_maps system).
 * For an example on how to use the same functions with the Polyhedron_3 class, see the function imposeConstraints from the TinCreationSimplificationPointSet class.
 *
 * @author Ricard Campos (ricardcd@gmail.com)
 */

// STD
#include <iostream>
#include <fstream>
#include <map>
// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
// Boost
#include <boost/program_options.hpp>
// Project-related
#include "cgal/detect_sharp_edges_without_borders.h"
#include "cgal/extract_polylines_from_sharp_edges.h"


typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef std::vector<Point_3> Polyline;

typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
typedef boost::graph_traits<Surface_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Surface_mesh>::edge_descriptor edge_descriptor;

typedef std::vector<Point_3>                                Polyline;
typedef std::vector<Polyline>                               Polylines;



using namespace std ;
namespace po = boost::program_options ;



int main( int argc, char** argv )
{
    std::string inputFile, outputFile;
    double angle;
    unsigned int minSize;
    po::options_description options("Detects sharp edges in a mesh without including border edges") ;
    options.add_options()
            ("help,h", "Produce help message")
            ("input,i", po::value<std::string>(&inputFile)->default_value(""), "Input mesh (OFF format)")
            ("output,o", po::value<std::string>(&outputFile)->default_value(""), "Output file containing a matlab script that creates the variable \"Edges\" (with the detected edges, one edge per line: x0 y0 z0 x1 y1 z1) and \"Polylines\" (the polylines derived from the edges, a cell array of a numPts x 3 matrices). When executed in matlab, it also plots the edges/polylines. If empty, the detected edges will be shown on screen with the same format.")
            ("angle,a", po::value<double>(&angle)->default_value(60), "Angle threshold (in degrees)")
            ("min-size,m", po::value<unsigned int>(&minSize)->default_value(0), "Minimum size")
    ;

    po::positional_options_description positionalOptions;
    positionalOptions.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(positionalOptions).run(), vm);
    po::notify(vm);

    if (inputFile.empty()) {
        cout << "Empty input file name, usage:" << std::endl;
    }

    if (inputFile.empty() || vm.count("help")) {
        cout << options << "\n";
        return 1;
    }

    std::ifstream is(inputFile);
    if(!is){
        std::cerr<< "Filename provided is invalid" << std::endl;
        return EXIT_FAILURE;
    }

    // Read the input file
    Surface_mesh sm;
    cout << "- Reading input file..." ;
    is >> sm  ;
    cout << " done." << endl ;

    // Detect sharp edges
    typedef typename Surface_mesh::Property_map<edge_descriptor, bool> EdgeIsSharpMap;
    EdgeIsSharpMap eisMap;
    bool created;
    boost::tie(eisMap, created) = sm.add_property_map<edge_descriptor, bool>("e:is_sharp", false);
    assert(created);

    detect_sharp_edges_without_borders<Surface_mesh, double, EdgeIsSharpMap, Kernel>(sm, angle, eisMap);

    // Output to file or std?
    std::ofstream ofs(outputFile);
    if (!outputFile.empty() && !ofs) {
        std::cerr << "Cannot open output file!" << std::endl;
        return EXIT_FAILURE;
    }
    ostream& output = outputFile.empty()? cout : ofs;

    // Output the detected sharp edges
    output << "Edges = [" << endl;
    int numSharpEdges = 0;
    BOOST_FOREACH(edge_descriptor ed, edges(sm)) {
        bool isSharp = get(eisMap, ed);
        if (isSharp) {
            // print the endpoints!
            output << sm.point(sm.vertex(ed, 0)) << " " << sm.point(sm.vertex(ed, 1)) << ";" << endl;
            numSharpEdges++;
        }
    }
    output << "];" << endl;

    cout << "- Num. Sharp Edges = " << numSharpEdges << endl;

    // Trace the polylines from the detected edges
    std::vector<Polyline> polylines;
    extract_polylines_from_sharp_edges(sm, eisMap, polylines);
    cout << "- Num. Polylines = " << polylines.size() << endl;

    // Output the polylines
    output << "Polylines = {" << endl;
    for (Polylines::iterator itPolys = polylines.begin(); itPolys != polylines.end(); ++itPolys) {
        if ((*itPolys).size() >= minSize) {
            output << "[" << endl;
            for (Polyline::iterator itp = (*itPolys).begin(); itp != (*itPolys).end(); ++itp) {
                output << *itp << ";" << endl;
            }
            output << "]," << endl;
        }
    }
    output << "};" << endl;

    // If the output matlab file is set, add a script to run visualize the detected edges and polylines
    if (!outputFile.empty()) {
        output << "figure;\n"
               << "title('Edges');\n"
               << "hold on;\n"
               << "for i = 1:size(Edges, 1)\n"
               << "\tplot3([Edges(i, 1) Edges(i, 4)],[Edges(i, 2) Edges(i, 5)],[Edges(i, 3) Edges(i, 6)]);\n"
               << "end\n"
               << "hold off;\n"
               << "axis equal;\n\n"
               << "figure;\n"
               << "title('Polylines');\n"
               << "hold on;\n"
               << "for i = 1:numel(Polylines)\n"
               << "\tpoly = Polylines{i};\n"
               << "\tfor j = 1:size(poly)\n"
               << "\t\tplot3(poly(:, 1), poly(:, 2), poly(:, 3));\n"
               << "\tend\n"
               << "end\n"
               << "hold off;\n"
               << "axis equal;\n";
    }

    return EXIT_SUCCESS;
}
