//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H

#include <memory>
#include "tin_creation_cgal_types.h"

// Note: this set of classes implement a Strategy Pattern
namespace TinCreation {

// Strategy base class
class TinCreationStrategy {
public:
    virtual Polyhedron create(const std::vector<Point_3> &dataPts,
                              const bool &constrainEasternVertices,
                              const bool &constrainWesternVertices,
                              const bool &constrainNorthernVertices,
                              const bool &constrainSouthernVertices) = 0;
};

// Context
class TinCreator {
public:
    // --- Methods ---
    TinCreator() { m_creator = nullptr; }

    void setCreator(std::shared_ptr<TinCreationStrategy> creator) { m_creator = creator; }

    Polyhedron create(const std::vector<Point_3> &dataPts,
                      const bool &constrainEasternVertices = false,
                      const bool &constrainWesternVertices = false,
                      const bool &constrainNorthernVertices = false,
                      const bool &constrainSouthernVertices = false) {
        return m_creator->create(dataPts,
                                 constrainEasternVertices,
                                 constrainWesternVertices,
                                 constrainNorthernVertices,
                                 constrainSouthernVertices);
    }

private:
    // --- Attributes ---
    std::shared_ptr<TinCreationStrategy> m_creator;
};

} // End namespace tin_creation

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
