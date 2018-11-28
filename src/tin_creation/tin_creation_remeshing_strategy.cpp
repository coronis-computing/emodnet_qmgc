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

#include "tin_creation_remeshing_strategy.h"
#include "cgal/generate_border_features_polylines.h"
#include <CGAL/make_mesh_3.h>
#include <iostream>
#include "cgal/polyhedron_builder_from_c3t3_boundary.h"
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
#include <CGAL/config.h>
#include "tin_creation_cgal_types.h"
#include <limits>
#include "cgal/extract_tile_borders_from_polyhedron.h"
#include <CGAL/bounding_box.h>

namespace TinCreation {

Polyhedron TinCreationRemeshingStrategy::create(const std::vector<Point_3> &dataPts,
                                                const bool &constrainEasternVertices,
                                                const bool &constrainWesternVertices,
                                                const bool &constrainNorthernVertices,
                                                const bool &constrainSouthernVertices) {
    using namespace CGAL::parameters;

    // First of all, check if the input data points are planar. If a planar mesh is input, the meshing algorithm never finishes!
    if (dataPtsArePlanar(dataPts)) {
        // Create a default triangulation for the grid and return
        std::vector<Point_3> defaultPts = defaultPointsForPlanarTile();

        // Delaunay triangulation
        Delaunay dt(defaultPts.begin(), defaultPts.end());

        // Translate to Polyhedron
        Polyhedron surface;
        PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
        surface.delegate(builderDT);

        return surface;
    }

    // Delaunay triangulation
    Delaunay dt(dataPts.begin(), dataPts.end());

    // Translate to Polyhedron
    Polyhedron surface;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
    surface.delegate(builderDT);

    // Convert the points to metric, but preserve the connectivity provided by the 2D Delaunay
    for (Polyhedron::Point_iterator it = surface.points_begin(); it != surface.points_end(); ++it)
        *it = this->convertUVHToECEF(*it);

    // Reset the origin of the points
    K::Iso_cuboid_3 boundingBox = CGAL::bounding_box(surface.points_begin(), surface.points_end());
////    std::transform(samples.begin(), samples.end(), samples.begin(),
////                   [boundingBox](Point_3& p){
////                       return Point_3(p.x()-boundingBox.xmin(),
////                                      p.y()-boundingBox.ymin(),
////                                      p.z()-boundingBox.zmin());
////                   }
////    );
    for (Polyhedron::Point_iterator it = surface.points_begin(); it != surface.points_end(); ++it)
        *it = Point_3((*it).x()-boundingBox.xmin(),
                      (*it).y()-boundingBox.ymin(),
                      (*it).z()-boundingBox.zmin());

    surface.normalize_border(); // Needed to detect the borders

    // Create a vector with only one element: the pointer to the polyhedron.
    std::vector<Polyhedron *> polyPtrsVector(1, &surface);

    // Create a polyhedral domain, with only one polyhedron,
    // and no "bounding polyhedron", so the volumetric part of the domain will be
    // empty.
    MeshDomain domain(polyPtrsVector.begin(), polyPtrsVector.end());

//    // Get the border polylines to maintain
//    Polylines polylines = generateBorderFeaturesPolylines<Polyhedron>(surface,
//                                                                      constrainEasternVertices,
//                                                                      constrainWesternVertices,
//                                                                      constrainNorthernVertices,
//                                                                      constrainSouthernVertices);
    // Extract the vertices on the borders
    PointCloud northernBorderVertices, southernBorderVertices, easternBorderVertices, westernBorderVertices ;
    Point_3 cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11;
    extractTileBordersFromPolyhedron<Polyhedron>(surface, easternBorderVertices, westernBorderVertices, northernBorderVertices, southernBorderVertices, cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11);
    // Sort the vertices vertically/horizontally so they form a polyline
    std::sort(easternBorderVertices.begin(), easternBorderVertices.end(), [](const Point_3& a, const Point_3& b) -> bool { return a.y() < b.y(); });
    std::sort(westernBorderVertices.begin(), westernBorderVertices.end(), [](const Point_3& a, const Point_3& b) -> bool { return a.y() < b.y(); });
    std::sort(northernBorderVertices.begin(), northernBorderVertices.end(), [](const Point_3& a, const Point_3& b) -> bool { return a.x() < b.x(); });
    std::sort(southernBorderVertices.begin(), southernBorderVertices.end(), [](const Point_3& a, const Point_3& b) -> bool { return a.x() < b.x(); });

    Polylines polylines;
    if (constrainEasternVertices) {
        Polylines edges = this->borderPolylineToIndividualEdges(easternBorderVertices);
        polylines.insert(polylines.end(), edges.begin(), edges.end() );
    }
    else {
        polylines.push_back(easternBorderVertices);
    }
    if (constrainWesternVertices) {
        Polylines edges = this->borderPolylineToIndividualEdges(westernBorderVertices);
        polylines.insert(polylines.end(), edges.begin(), edges.end() );
    }
    else {
        polylines.push_back(westernBorderVertices);
    }
    if (constrainNorthernVertices) {
        Polylines edges = this->borderPolylineToIndividualEdges(northernBorderVertices);
        polylines.insert(polylines.end(), edges.begin(), edges.end() );
    }
    else {
        polylines.push_back(northernBorderVertices);
    }
    if (constrainSouthernVertices) {
        Polylines edges = this->borderPolylineToIndividualEdges(southernBorderVertices);
        polylines.insert(polylines.end(), edges.begin(), edges.end() );
    }
    else {
        polylines.push_back(southernBorderVertices);
    }

    std::cout << "Adding features, imposing " << polylines.size() << " polylines" << std::endl ;
    domain.add_features(polylines.begin(), polylines.end());

    // Mesh criteria
    // WARNING: Manifold criteria in mesh_3 is an undocumented feature as of CGAL 4.9.
    // In fact, the documented feature FACET_VERTICES_ON_SAME_SURFACE_PATCH_WITH_ADJACENCY_CHECK is not implemented!
    // See: https://github.com/CGAL/cgal/pull/590, and CGAL/Mesh_facet_topology.h
    MeshCriteria criteria(CGAL::parameters::edge_size = m_edgeSize,
                          CGAL::parameters::facet_angle = m_facetAngle,
                          CGAL::parameters::facet_size = m_facetSize,
                          CGAL::parameters::facet_distance = m_facetDistance,
                          CGAL::parameters::facet_topology = CGAL::MANIFOLD_WITH_BOUNDARY);

//    std::cout << "Meshing criteria:" << std::endl ;
//    std::cout << "    - edge_size = " << m_edgeSize << std::endl ;
//    std::cout << "    - facet_angle = " << m_facetAngle << std::endl ;
//    std::cout << "    - facet_size = " << m_facetSize << std::endl ;
//    std::cout << "    - facet_distance = " << m_facetDistance << std::endl ;
//    std::cout << "    - facet_topology = CGAL::MANIFOLD_WITH_BOUNDARY" << std::endl ;

    // Mesh generation
    C3T3 c3t3 = CGAL::make_mesh_3<C3T3>(domain, criteria, no_perturb(), no_exude());

    // Extract the surface boundary as a polyhedron
    Polyhedron remeshedSurface;
    PolyhedronBuilderFromC3T3Boundary<C3T3, HalfedgeDS> builderC3T3Boundary(c3t3, 0);
    remeshedSurface.delegate(builderC3T3Boundary);

    // Convert back the points to UVH
    for (Polyhedron::Point_iterator it = remeshedSurface.points_begin(); it != remeshedSurface.points_end(); ++it)
        *it = this->convertECEFToUVH(*it);

    return remeshedSurface;
}


bool TinCreationRemeshingStrategy::dataPtsArePlanar(const std::vector<Point_3> &dataPts) const {
    for (std::vector<Point_3>::const_iterator it = dataPts.begin(); it != dataPts.end(); ++it) {
        if (it->z() > std::numeric_limits<double>::epsilon())
            return false;
    }

    return true;
}


std::vector<Point_3> TinCreationRemeshingStrategy::defaultPointsForPlanarTile() const {
    std::vector<Point_3> pts;

    double step = 0.2;
    for (double i = 0; i <= 1; i += step) {
        for (double j = 0; j <= 1; j += step) {
            Point_3 p(i, j, 0);
            pts.push_back(p);
        }
    }

    return pts;
}


Polylines TinCreationRemeshingStrategy::borderPolylineToIndividualEdges(Polyline& poly)
{
    // Collect the edges along the polyline
    Polylines edges;
    for (int i = 0; i < (poly.size()-1); i++) {
        Polyline edge;
        edge.push_back(poly[i]);
        edge.push_back(poly[i+1]);
        edges.push_back(edge);
    }

    return edges;
}

} // End namespace TinCreation