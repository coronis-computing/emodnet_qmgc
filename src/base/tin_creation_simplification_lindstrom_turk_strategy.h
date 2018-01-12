//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_SURFACE_SIMPLIFICATION_LINDSTROM_TURK_STRATEGY_H
#define EMODNET_TOOLS_SURFACE_SIMPLIFICATION_LINDSTROM_TURK_STRATEGY_H

#include "tin_creator.h"

/**
 * Simplifies the surface using Lindstrom-Turk algorithm [1][2]
 * [1] Peter Lindstrom and Greg Turk. Fast and memory efficient polygonal simplification. In IEEE Visualization, pages 279–286, 1998.
 * [2] P. Lindstrom and G. Turk. Evaluation of memoryless simplification. IEEE Transactions on Visualization and Computer Graphics, 5(2):98–115, slash 1999.
 */
class TINCreationSimplificationLindstromTurkStrategy : public TINCreationStrategy
{
public:
    TINCreationSimplificationLindstromTurkStrategy( int stopEdgesCount,
                                                double weightVolume = 0.5,
                                                double weightBoundary = 0.5,
                                                double weightShape = 1e-10)
            : m_stopEdgesCount(stopEdgesCount)
            , m_weightVolume(weightVolume)
            , m_weightBoundary(weightBoundary)
            , m_weightShape(weightShape) {}

    Polyhedron create( const std::vector<Point_3>& dataPts,
                       const bool& constrainEasternVertices,
                       const bool& constrainWesternVertices,
                       const bool& constrainNorthernVertices,
                       const bool& constrainSouthernVertices ) const ;
private:
    // Algorithm parameters
    int m_stopEdgesCount ;    // Simplification edges count stop condition. If the number of edges in the surface being simplified drops below this threshold the process finishes
    double m_weightVolume ;   // Weight for the volume part of Lindstrom-Turk's cost function
    double m_weightBoundary ; // Weight for the boundary part of Lindstrom-Turk's cost function
    double m_weightShape ;    // Weight for the shape part of Lindstrom-Turk's cost function
};


#endif //EMODNET_TOOLS_SURFACE_SIMPLIFICATION_LINDSTROM_TURK_STRATEGY_H
