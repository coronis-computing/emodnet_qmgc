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

#ifndef EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
#define EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_H

#include "tin_creator.h"
#include "tin_creation_cgal_types.h"
#include "cgal/Projection_traits_3_extended.h"
#include "cgal/point_set_features_simplification_cost.h"
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class TinCreationSimplificationPointSet
 * @brief Creates a TIN using a point set simplification algorithm
 *
 * This is an interphase class for point set simplification methods. In this class, the main processing required to use
 * point set simplification techniques in square tiles is defined, but the actual point set simplification method is defined
 * in child classes, one for each method.
 *
 * Thus, this class is responsible of running the common part of these methods, which
 * is basically to detect border and feature edges in the original data, create polylines from those, and simplify them
 * separately. Once the polylines have been simplified, the rest of point sets are simplified (using the method implemented
 * in each child class) and triangulated in the XY plane.
 */
class TinCreationSimplificationPointSet : public TinCreationStrategy
{
    // --- Typedefs ---
    typedef CGAL::Projection_traits_xy_3_extended<K>                         ProjTraitsXY;
    typedef PS::Vertex_base_2<ProjTraitsXY>                                  VbXY;
    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsXY>        FbXY;
    typedef CGAL::Triangulation_data_structure_2<VbXY, FbXY>                 TDSXY;
    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsXY,
            TDSXY, CGAL::Exact_predicates_tag>                               CDTXY;
    typedef CGAL::Constrained_triangulation_plus_2<CDTXY>                    CTXY;
    typedef PS::PointSetFeaturesSimplificationCost                                      PSSqDist3Cost;

public:
    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     */
    TinCreationSimplificationPointSet(double borderSimplificationMaxDistance,
                                      double borderSimplificationMaxLengthPercent,
                                      unsigned int minFeaturePolylineSize,
                                      bool preserveSharpEdges = true)
            : m_minFeaturePolylineSize(minFeaturePolylineSize)
            , m_preserveSharpEdges(preserveSharpEdges)
    {
        m_borderSimpMaxDistPerZoom = std::vector<FT>{borderSimplificationMaxDistance};
        m_borderSimpMaxLengthPercentPerZoom = std::vector<FT>{borderSimplificationMaxLengthPercent};
//        setParamsForZoom(0);
    }

    /**
     * Constructor
     * @param borderSimplificationMaxDistance Maximum error for polyline simplification per zoom
     * @param borderSimplificationMaxLengthPercent Maximum length for an edge in the simplified polyline per zoom. This prevents oversimplification in planar tiles.
     * @param minFeaturePolylineSize Minimum number of connected edges in a sharp feature polyline to consider it during processing
     */
    TinCreationSimplificationPointSet(const std::vector<double>& borderSimplificationMaxDistance,
                                      const std::vector<double>& borderSimplificationMaxLengthPercent,
                                      unsigned int minFeaturePolylineSize,
                                      bool preserveSharpEdges = true)
            : m_borderSimpMaxDistPerZoom(borderSimplificationMaxDistance)
            , m_borderSimpMaxLengthPercentPerZoom(borderSimplificationMaxLengthPercent)
            , m_preserveSharpEdges(preserveSharpEdges)
    {
//        setParamsForZoom(0);
    }

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool &constrainEasternVertices = false,
                      const bool &constrainWesternVertices = false,
                      const bool &constrainNorthernVertices = false,
                      const bool &constrainSouthernVertices = false) ;

    /**
     * Simplifies a point set
     * @param pts Points to simplify
     * @return Simplified point set
     */
    // While the process of preserving border/feature edges is implemented in this base class, the methods using point
    // set simplification differ on the way they simplify the point set
    // Thus, this is the only method that needs to be implemented in the child classes
    virtual std::vector<Point_3> simplify(const std::vector<Point_3>& pts) = 0;

    void setParamsForZoom(const unsigned int& zoom) {
        m_borderSimpMaxDist = standardHandlingOfThresholdPerZoom(m_borderSimpMaxDistPerZoom, zoom);

        m_borderSimpMaxLengthPercent = standardHandlingOfThresholdPerZoom(m_borderSimpMaxLengthPercentPerZoom, zoom);
        m_borderSimpMaxLengthPercent /= 100.0; // Note that the m_borderSimpMaxLengthPercent is a percentage, so we divide it by 100 to have it between 0..1, as expected by the CGAL::Polyline_simplification_2::simplify function

        // Set further parameters that are exclusive of each point set simplification strategy using setParamsForZoomConcreteStrategy(zoom); in derived classes
        setParamsForZoomConcreteStrategy(zoom);
    }

    /**
     * Same as setParamsForZoom, but triggered at each child class
     * @param zoom Current zoom level
     */
    virtual void setParamsForZoomConcreteStrategy(const unsigned int& zoom) = 0;

private:
    double m_borderSimpMaxDist;
    double m_borderSimpMaxScaledSqDist;
    double m_borderSimpMaxLengthPercent; // We do not scale this length, as it is relative to the XY plane
    std::vector<double> m_borderSimpMaxDistPerZoom;
    std::vector<double> m_borderSimpMaxLengthPercentPerZoom;
    unsigned int m_minFeaturePolylineSize;
    CTXY m_cdt;
    bool m_preserveSharpEdges;

    /// Imposes the required constraints to the internal CDT structure. Simplified the border/feature polylines when needed
    void imposeConstraintsAndSimplifyPolylines(Polyhedron& surface,
                                               const bool &constrainEasternVertices,
                                               const bool &constrainWesternVertices,
                                               const bool &constrainNorthernVertices,
                                               const bool &constrainSouthernVertices) ;

    /// Gets all the vertices not on the borders of the polyhedron
    /// Precondition: poly.normalize_border() has been executed, and is valid
    void getAllNonBorderVertices(const Polyhedron& poly, PointCloud& nonBorderPts) const ;
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_SIMPLIFICATION_POINT_SET_H
