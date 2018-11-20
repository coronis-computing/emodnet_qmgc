//
// Author: Ricard Campos (ricardcd@gmail.com)
//

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