//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set_random.h"
#include <CGAL/random_simplify_point_set.h>



std::vector<Point_3>
TinCreationSimplificationPointSetRandom::
simplify(const std::vector<Point_3> &pts)
{
    std::vector<Point_3> ptsSimp = pts ;
    ptsSimp.erase(CGAL::random_simplify_point_set(ptsSimp.begin(),
                                                  ptsSimp.end(),
                                                  m_removePercentage),
                  ptsSimp.end());
    return ptsSimp ;
}