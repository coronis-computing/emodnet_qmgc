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

#include "tin_creation_simplification_point_set_wlop.h"
#include <CGAL/wlop_simplify_and_regularize_point_set.h>

namespace TinCreation {

std::vector<Point_3>
TinCreationSimplificationPointSetWLOP::
simplify(const std::vector<Point_3> &pts) {
    // Convert to metric
    std::vector<Point_3> ptsToSimpECEF = this->convertUVHToECEF(pts);

    std::vector<Point_3> ptsSimpECEF;
    CGAL::wlop_simplify_and_regularize_point_set
            <Concurrency_tag>
            (ptsToSimpECEF.begin(),
             ptsToSimpECEF.end(),
             std::back_inserter(ptsSimpECEF),
             m_retainPercentage,
             m_radius
            );

    // Convert to the local (XY-projectable) coordinates again
    std::vector<Point_3> ptsSimp = this->convertECEFToUVH(ptsSimpECEF);

    return ptsSimp;
}

} // End namespace TinCreation