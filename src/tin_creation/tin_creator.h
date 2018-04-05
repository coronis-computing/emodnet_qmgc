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

    TinCreationStrategy()
            : m_scaleZ(1.0)
            , m_minX(-1.0), m_minY(-1.0), m_minZ(-1.0), m_maxX(-1.0), m_maxY(-1.0), m_maxZ(-1.0) {}

    virtual Polyhedron create(const std::vector<Point_3> &dataPts,
                              const bool &constrainEasternVertices,
                              const bool &constrainWesternVertices,
                              const bool &constrainNorthernVertices,
                              const bool &constrainSouthernVertices) = 0;

    virtual void setParamsForZoom(const unsigned int& zoom) = 0;

    void setScaleZ(const double& scale) { m_scaleZ = scale; }
    double getScaleZ() { return m_scaleZ; }

    void setBounds(const double& minX, const double& minY, const double& minZ,
                   const double& maxX, const double& maxY, const double& maxZ ) {
        m_minX = minX; m_minY = minY; m_minZ = minZ;
        m_maxX = maxX; m_maxY = maxY; m_maxZ = maxZ;
    }

    double getMinX() const { return m_minX; }
    double getMinY() const { return m_minY; }
    double getMinZ() const { return m_minZ; }
    double getMaxX() const { return m_maxX; }
    double getMaxY() const { return m_maxY; }
    double getMaxZ() const { return m_maxZ; }

private:
    double m_scaleZ;
    double m_minX, m_minY, m_minZ, m_maxX, m_maxY, m_maxZ;
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

    void setParamsForZoom(const unsigned int& zoom) {
        m_creator->setParamsForZoom(zoom);
    }

    void setScaleZ(const double& scale) { m_creator->setScaleZ(scale); }
    void setBounds(const double& minX, const double& minY, const double& minZ,
                   const double& maxX, const double& maxY, const double& maxZ ) {
        m_creator->setBounds(minX, minY, minZ, maxX, maxY, maxZ);
    }

private:
    // --- Attributes ---
    std::shared_ptr<TinCreationStrategy> m_creator;
};

} // End namespace tin_creation

#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_STRATEGY_H
