//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set_hierarchy.h"
#include <CGAL/hierarchy_simplify_point_set.h>



std::vector<Point_3>
TinCreationSimplificationPointSetHierarchy::
simplify(const std::vector<Point_3> &pts)
{
    std::vector<Point_3> ptsSimp = pts ;
    ptsSimp.erase(CGAL::hierarchy_simplify_point_set (ptsSimp.begin(),
                                                      ptsSimp.end(),
                                                      m_maxClusterSize, // Max cluster size
                                                      m_maxSurfaceVariance), // Max surface variation
                  ptsSimp.end());
    return ptsSimp ;
}