//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
#define EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H

#include "tin_creation_simplification_point_set.h"

namespace TinCreation {

class TinCreationSimplificationPointSetGrid
        : public TinCreationSimplificationPointSet
{
public:
    TinCreationSimplificationPointSetGrid(double borderSimplificationMaxDistance,
                                          double borderSimplificationMaxLength,
                                          unsigned int minFeaturePolylineSize,
                                          double cellSize)
            : TinCreationSimplificationPointSet(borderSimplificationMaxDistance, borderSimplificationMaxLength, minFeaturePolylineSize)
            , m_cellSize(cellSize) {}

    std::vector<Point_3> simplify(const std::vector<Point_3>& pts);

private:
    double m_cellSize ;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_SIMPLIFICATION_POINT_SET_GRID_H
