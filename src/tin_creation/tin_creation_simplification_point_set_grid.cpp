//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set_grid.h"
#include <CGAL/grid_simplify_point_set.h>

namespace TinCreation {

std::vector<Point_3>
TinCreationSimplificationPointSetGrid::
simplify(const std::vector<Point_3> &pts) {
    // Convert to metric
    std::vector<Point_3> ptsToSimpECEF = this->convertUVHToECEF(pts);

//    std::cout << "PointsECEF = [ " << std::endl;
//    for( std::vector<Point_3>::iterator it = ptsToSimpECEF.begin(); it != ptsToSimpECEF.end(); ++it ) {
//        std::cout << *it << std::endl;
//    }
//    std::cout << "];" << std::endl;

    // Simplify using grid simplification (erase-remove idiom)
    ptsToSimpECEF.erase(CGAL::grid_simplify_point_set(ptsToSimpECEF.begin(),
                                                      ptsToSimpECEF.end(),
                                                      m_cellSize),
                        ptsToSimpECEF.end());

    // Convert to the local (XY-projectable) coordinates again
    std::vector<Point_3> ptsSimp = this->convertECEFToUVH(ptsToSimpECEF);

//    std::cout << "PointsSimp = [ " << std::endl;
//    for( std::vector<Point_3>::iterator it = ptsSimp.begin(); it != ptsSimp.end(); ++it ) {
//        std::cout << *it << std::endl;
//    }
//    std::cout << "];" << std::endl;

    return ptsSimp;
}

} // End namespace TinCreation