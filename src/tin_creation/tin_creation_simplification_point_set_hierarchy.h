//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

class TinCreationSimplificationPointSetHierarchy
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetHierarchy(double borderSimplificationMaxDistance,
                                               double borderSimplificationMaxLength,
                                               unsigned int minFeaturePolylineSize,
                                               double maxClusterSize,
                                               double maxSurfaceVariance)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, borderSimplificationMaxLength, minFeaturePolylineSize)
            , m_maxClusterSize(maxClusterSize)
            , m_maxSurfaceVariance(maxSurfaceVariance) {}

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

private:
    unsigned int m_maxClusterSize;
    double m_maxSurfaceVariance;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_HIERARCHY_H
