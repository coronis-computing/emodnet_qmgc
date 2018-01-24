//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

class TinCreationSimplificationPointSetWLOP
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetWLOP(double borderSimplificationMaxDistance,
                                          unsigned int minFeaturePolylineSize,
                                          double retainPercentage,
                                          double radius,
                                          unsigned int iterNumber = 35)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, minFeaturePolylineSize)
            , m_retainPercentage(retainPercentage)
            , m_radius(radius)
            , m_iterNumber(iterNumber) {}

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

private:
    double m_retainPercentage;
    double m_radius;
    unsigned int m_iterNumber;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_WLOP_H
