//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFICATION_VOID_CONCRETE_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFICATION_VOID_CONCRETE_STRATEGY_H

#include "surface_simplifier.h"
#include <iostream>

/**
 * Void simplification
 * Basically, this means no simplification is applied
 */

class SurfaceSimplificationVoidStrategy : public SurfaceSimplificationStrategy
{
public:
    SurfaceSimplificationVoidStrategy() {} ;

    void simplify(Polyhedron &surface,
                  const bool &constrainEasternVertices,
                  const bool &constrainWesternVertices,
                  const bool &constrainNorthernVertices,
                  const bool &constrainSouthernVertices) const {}
} ;

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_VOID_CONCRETE_STRATEGY_H
