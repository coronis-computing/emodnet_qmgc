//
// Author: Ricard Campos (ricardcd@gmail.com)
//

#include "tin_creation_simplification_point_set_hierarchy.h"
#include <CGAL/hierarchy_simplify_point_set.h>
#include <iostream>

namespace TinCreation {

std::vector<Point_3>
TinCreationSimplificationPointSetHierarchy::
simplify(const std::vector<Point_3> &pts) {
    // Convert to metric
    std::vector<Point_3> ptsToSimpECEF = this->convertUVHToECEF(pts);

    // Simplify using hierarchical point set simplification (erase-remove idiom)
    ptsToSimpECEF.erase(CGAL::hierarchy_simplify_point_set(ptsToSimpECEF.begin(),
                                                           ptsToSimpECEF.end(),
                                                           m_maxClusterSize, // Max cluster size
                                                           m_maxSurfaceVariance), // Max surface variation
                        ptsToSimpECEF.end());

    // Convert to the local (XY-projectable) coordinates again
    std::vector<Point_3> ptsSimp = this->convertECEFToUVH(ptsToSimpECEF);

    return ptsSimp;
}

} // End namespace TinCreation