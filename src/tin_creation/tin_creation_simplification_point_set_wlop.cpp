//
// Author: Ricard Campos (ricardcd@gmail.com)
//

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