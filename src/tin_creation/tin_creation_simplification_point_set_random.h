//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

class TinCreationSimplificationPointSetRandom
        : public TinCreationSimplificationPointSet {
public:
    TinCreationSimplificationPointSetRandom(double borderSimplificationMaxDistance,
                                            double borderSimplificationMaxLength,
                                            unsigned int minFeaturePolylineSize,
                                            double removePercentage)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, borderSimplificationMaxLength, minFeaturePolylineSize),
              m_removePercentage(removePercentage) {}

    std::vector<Point_3> simplify(const std::vector<Point_3> &pts);

private:
    double m_removePercentage;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_RANDOM_H
