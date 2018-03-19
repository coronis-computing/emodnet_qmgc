//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set_wlop.h"
#include <CGAL/wlop_simplify_and_regularize_point_set.h>

namespace TinCreation {

std::vector<Point_3>
TinCreationSimplificationPointSetWLOP::
simplify(const std::vector<Point_3> &pts) {
    std::vector<Point_3> ptsOrig = pts, ptsSimp;
    //parameters
//    const double retain_percentage = 2;   // percentage of points to retain.
//    const double neighbor_radius = 0.5;   // neighbors size.
    CGAL::wlop_simplify_and_regularize_point_set
            <Concurrency_tag>
            (ptsOrig.begin(),
             ptsOrig.end(),
             std::back_inserter(ptsSimp),
             m_retainPercentage,
             m_radius
            );

    return ptsSimp;
}

} // End namespace TinCreation