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

#include "tin_creation_delaunay_strategy.h"
#include "tin_creation_cgal_types.h"
#include "cgal/polyhedron_builder_from_projected_triangulation.h"

namespace TinCreation {

Polyhedron TinCreationDelaunayStrategy::create(const std::vector<Point_3> &dataPts,
                                               const bool &constrainEasternVertices,
                                               const bool &constrainWesternVertices,
                                               const bool &constrainNorthernVertices,
                                               const bool &constrainSouthernVertices) {
    // Delaunay triangulation
    Delaunay dt(dataPts.begin(), dataPts.end());

    // Translate to Polyhedron
    Polyhedron surface;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    return surface;
}

} // End namespace TinCreation