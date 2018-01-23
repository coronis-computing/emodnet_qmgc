//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set_grid.h"
#include <CGAL/grid_simplify_point_set.h>



std::vector<Point_3>
TinCreationSimplificationPointSetGrid::
simplify(const std::vector<Point_3>& pts)
{
    std::vector<Point_3> ptsSimp = pts ;
    ptsSimp.erase(CGAL::grid_simplify_point_set(ptsSimp.begin(),
                                                ptsSimp.end(),
                                                m_cellSize),
                  ptsSimp.end());
    return ptsSimp ;
}
