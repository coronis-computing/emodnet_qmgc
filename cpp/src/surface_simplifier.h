//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H

#include "cgal_defines.h"
// Note: this set of classes implement a Strategy Pattern

// Strategy base class
class SurfaceSimplificationStrategy
{
public:
    virtual void simplify( Polyhedron& surface,
                           const bool& constrainEasternVertices,
                           const bool& constrainWesternVertices,
                           const bool& constrainNorthernVertices,
                           const bool& constrainSouthernVertices ) const = 0 ;
};


// Context
class SurfaceSimplifier
{
public:
    SurfaceSimplifier() {}

    /// Allows to change the simplification algorithm at runtime
    void setSimplifier( SurfaceSimplificationStrategy* simplifier ) { m_simplifier = simplifier ; }

    void simplify( Polyhedron& surface,
                   const bool& constrainEasternVertices,
                   const bool& constrainWesternVertices,
                   const bool& constrainNorthernVertices,
                   const bool& constrainSouthernVertices ) const
    {
        m_simplifier->simplify( surface,
                                constrainEasternVertices,
                                constrainWesternVertices,
                                constrainNorthernVertices,
                                constrainSouthernVertices ) ;
    }

private:
    SurfaceSimplificationStrategy* m_simplifier ;
};

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
