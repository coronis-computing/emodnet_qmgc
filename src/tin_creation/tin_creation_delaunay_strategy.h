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

#ifndef EMODNET_QMGC_TIN_CREATION_DELAUNAY_H
#define EMODNET_QMGC_TIN_CREATION_DELAUNAY_H

#include "tin_creation/tin_creator.h"

namespace TinCreation {

/**
 * @class TinCreationDelaunayStrategy
 * @brief A Delaunay triangulation is created with the input points, no simplification is applied.
 *
 * Also, the constrain<X>Vertices parameters are ignored. This creation strategy is useful to just triangulate regular
 * grids, where the vertices at the borders are always the same for neighboring tiles.
 */
class TinCreationDelaunayStrategy : public TinCreationStrategy {
public:
    TinCreationDelaunayStrategy() {};

    Polyhedron create(const std::vector<Point_3> &dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices);

    void setParamsForZoom(const unsigned int& zoom) {}
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_DELAUNAY_H
