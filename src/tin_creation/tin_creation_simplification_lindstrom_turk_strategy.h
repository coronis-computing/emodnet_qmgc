// Copyright (c) 2018 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is part of EMODnet Quantized Mesh Generator for Cesium.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Author: Ricard Campos (ricardcd@gmail.com)

#ifndef EMODNET_QMGC_TIN_CREATION_LINDSTROM_TURK_STRATEGY_H
#define EMODNET_QMGC_TIN_CREATION_LINDSTROM_TURK_STRATEGY_H

#include "tin_creator.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationLindstromTurkStrategy
 *
 * @brief Creates a TIN using an edge-collapse simplification method
 *
 * This class uses a modified version of the Lindstrom-Turk algorithm [1][2]
 *
 * [1] P. Lindstrom and G. Turk. Fast and memory efficient polygonal simplification. In IEEE Visualization, pages 279–286, 1998. <br>
 * [2] P. Lindstrom and G. Turk. Evaluation of memoryless simplification. IEEE Transactions on Visualization and Computer Graphics, 5(2):98–115, slash 1999.
 */
class TinCreationSimplificationLindstromTurkStrategy : public TinCreationStrategy {
public:
    /**
     * Constructor
     * @param stopEdgesCount Desired number of edges for the simplified mesh
     * @param weightVolume Volume weight (see original reference)
     * @param weightBoundary Boundary weight (see original reference)
     * @param weightShape Shape weight (see original reference)
     */
    TinCreationSimplificationLindstromTurkStrategy(int stopEdgesCount,
                                                   double weightVolume = 0.5,
                                                   double weightBoundary = 0.5,
                                                   double weightShape = 1e-10)
            : m_stopEdgesCount(stopEdgesCount)
            , m_weightVolume(weightVolume)
            , m_weightBoundary(weightBoundary)
            , m_stopEdgesCountPerZoom(1, stopEdgesCount)
            , m_weightShape(weightShape) {}

    /**
     * Constructor
     * @param stopEdgesCount Desired number of edges for the simplified mesh per zoom
     * @param weightVolume Volume weight (see original reference)
     * @param weightBoundary Boundary weight (see original reference)
     * @param weightShape Shape weight (see original reference)
     */
    TinCreationSimplificationLindstromTurkStrategy(std::vector<int> stopEdgesCountPerZoom,
                                                   double weightVolume = 0.5,
                                                   double weightBoundary = 0.5,
                                                   double weightShape = 1e-10)
            : m_stopEdgesCountPerZoom(stopEdgesCountPerZoom), m_weightVolume(weightVolume), m_weightBoundary(weightBoundary),
              m_weightShape(weightShape)
    {
        setParamsForZoom(0);
    }

    void setParamsForZoom(const unsigned int& zoom) {
        if (m_stopEdgesCountPerZoom.size() == 0) {
            std::cerr << "[WARNING::TinCreationSimplificationLindstromTurkStrategy] Input edges count per zoom vector is empty, using 500 (default value)" << std::endl;
            m_stopEdgesCount = 500;
        }
        else if (zoom < m_stopEdgesCountPerZoom.size())
            // Use the edges count corresponding to the required zoom
            m_stopEdgesCount = m_stopEdgesCountPerZoom[zoom];
        else {
            // Use the edges count corresponding to the last zoom specified in the vector
            m_stopEdgesCount = m_stopEdgesCountPerZoom.back();
        }
    }

    Polyhedron create(const std::vector<Point_3> &dataPts,
                      const bool &constrainEasternVertices,
                      const bool &constrainWesternVertices,
                      const bool &constrainNorthernVertices,
                      const bool &constrainSouthernVertices);

private:
    // Algorithm parameters
    int m_stopEdgesCount;    // Simplification edges count stop condition. If the number of edges in the surface being simplified drops below this threshold the process finishes
    double m_weightVolume;   // Weight for the volume part of Lindstrom-Turk's cost function
    double m_weightBoundary; // Weight for the boundary part of Lindstrom-Turk's cost function
    double m_weightShape;    // Weight for the shape part of Lindstrom-Turk's cost function
    std::vector<int> m_stopEdgesCountPerZoom; // Vector of desired edges count per zoom level
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_LINDSTROM_TURK_STRATEGY_H
