//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H

#include "tin_creation_simplification_point_set.h"


class TinCreationSimplificationPointSetRandom
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetRandom(double borderSimplificationMaxDistance,
                                            unsigned int minFeaturePolylineSize,
                                            double removePercentage)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, minFeaturePolylineSize)
            , m_removePercentage(removePercentage) {}

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

private:
    double m_removePercentage ;
};

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
