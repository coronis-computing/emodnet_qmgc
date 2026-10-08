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

    if (ptsToSimpECEF.size() == 0)
        return std::vector<Point_3>();

    std::vector<Point_3> ptsSimpECEF;
    // Note: since CGAL 5.x this takes the point range itself, and its tuning
    // arguments are named parameters rather than positional ones.
    //
    // m_iterNumber is passed explicitly here: it is set from the
    // --tc-ps-wlop-iter-number option but was never forwarded to CGAL, so that
    // option silently did nothing. CGAL's default matches the option's default
    // of 35, so wiring it up leaves default runs unchanged.
    CGAL::wlop_simplify_and_regularize_point_set
            <Concurrency_tag>
            (ptsToSimpECEF,
             std::back_inserter(ptsSimpECEF),
             CGAL::parameters::select_percentage(m_retainPercentage)
                     .neighbor_radius(m_radius)
                     .number_of_iterations(m_iterNumber)
            );

    // Convert to the local (XY-projectable) coordinates again
    std::vector<Point_3> ptsSimp = this->convertECEFToUVH(ptsSimpECEF);

    return ptsSimp;
}

} // End namespace TinCreation