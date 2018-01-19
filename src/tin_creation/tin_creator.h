//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H

#include "cgal_defines.h"
// Note: this set of classes implement a Strategy Pattern

// Strategy base class
class TINCreationStrategy
{
public:
    virtual Polyhedron create( const std::vector<Point_3>& dataPts,
                               const bool& constrainEasternVertices,
                               const bool& constrainWesternVertices,
                               const bool& constrainNorthernVertices,
                               const bool& constrainSouthernVertices ) = 0 ;
};


// Context
class TINCreator
{
public:
    TINCreator() {}
//    TINCreator(const TINCreator& tc) {m_creator.reset(tc.m_creator) ;}

    /// Allows to change the TIN creation algorithm at runtime
//    void setCreator( TINCreationStrategy* creator ) { m_creator.reset(creator) ; }
    void setCreator( TINCreationStrategy* creator ) { m_creator = creator ; }

    Polyhedron create( const std::vector<Point_3>& dataPts,
                       const bool& constrainEasternVertices,
                       const bool& constrainWesternVertices,
                       const bool& constrainNorthernVertices,
                       const bool& constrainSouthernVertices )
    {
        return m_creator->create( dataPts,
                                  constrainEasternVertices,
                                  constrainWesternVertices,
                                  constrainNorthernVertices,
                                  constrainSouthernVertices ) ;
    }

private:
//    std::unique_ptr<TINCreationStrategy> m_creator ;
    TINCreationStrategy* m_creator ;
};

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
