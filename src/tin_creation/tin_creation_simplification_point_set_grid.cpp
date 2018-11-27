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

#include "tin_creation_simplification_point_set_grid.h"
#include <CGAL/grid_simplify_point_set.h>

namespace TinCreation {

std::vector<Point_3>
TinCreationSimplificationPointSetGrid::
simplify(const std::vector<Point_3> &pts) {
    // Convert to metric
    std::vector<Point_3> ptsToSimpECEF = this->convertUVHToECEF(pts);

    // Simplify using grid simplification (erase-remove idiom)
    ptsToSimpECEF.erase(CGAL::grid_simplify_point_set(ptsToSimpECEF.begin(),
                                                      ptsToSimpECEF.end(),
                                                      m_cellSize),
                        ptsToSimpECEF.end());

    // Convert to the local (XY-projectable) coordinates again
    std::vector<Point_3> ptsSimp = this->convertECEFToUVH(ptsToSimpECEF);

    return ptsSimp;
}

} // End namespace TinCreation