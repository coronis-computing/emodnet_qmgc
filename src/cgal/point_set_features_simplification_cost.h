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

#ifndef EMODNET_QMGC_POINT_SET_FEATURES_SIMPLIFICATION_COST_H
#define EMODNET_QMGC_POINT_SET_FEATURES_SIMPLIFICATION_COST_H

#include <CGAL/algorithm.h>
#include <CGAL/intersections.h>

namespace CGAL {

template < class Tr >
class Constrained_triangulation_plus_2;

namespace Polyline_simplification_2
{

/**
 * @class PointSetFeaturesSimplificationCost
 * @brief Polyline simplification cost function used by point set simplification methods
 *
 * This class is a cost function which calculates the cost as the square of the distance between the original and simplified polylines in height.
 * It also allows to fix a maximum length on the XY plane for simplified edges
 * To be used with Projection_traits_xy_3 class.
 */
class PointSetFeaturesSimplificationCost
{

public:

    /// Initializes the cost function
    PointSetFeaturesSimplificationCost(const double& maxLength) : m_maxSqLength(maxLength*maxLength) {}

    /// Given a vertex in constraint iterator `vicq` computes `vicp=std::prev(vicq)` and `vicr=std::next(vicq)`,
    /// returns the maximum of the square distances between each point along the original subpolyline,
    /// between `vicp` and `vicr`, and the straight line segment  from `*vicp->point() to *vicr->point()`.
    /// \tparam CDT  must be `CGAL::Constrained_Delaunay_triangulation_2` with a vertex type that
    /// is model of  `PolylineSimplificationVertexBase_2`.
    template<class CDT>
    boost::optional<typename CDT::Geom_traits::FT>
    operator()(const Constrained_triangulation_plus_2<CDT>& pct
               , typename Constrained_triangulation_plus_2<CDT>::Vertices_in_constraint_iterator vicq)const
    {
        // Typedefs for projected calculations
        typedef typename Constrained_triangulation_plus_2<CDT>::Points_in_constraint_iterator Points_in_constraint_iterator;
        typedef typename Constrained_triangulation_plus_2<CDT>::Geom_traits Geom_traits ;
        typedef typename Geom_traits::FT                                  FT;
        typedef typename Geom_traits::Compute_squared_distance_3 Compute_squared_distance;
        typedef typename Geom_traits::Construct_segment_3        Construct_segment;
        typedef typename Geom_traits::Segment_3                  Segment;
        typedef typename Geom_traits::Point_3                    Point;

        // Typedefs for unprojected calculations
        typedef typename Geom_traits::Rp::Vector_3               Vector_3;
        typedef typename Geom_traits::Rp::Plane_3                   Plane_3;
        typedef typename Geom_traits::Rp::Line_3                    Line_3;
        typedef typename Geom_traits::Rp::Segment_3                 Segment_3;
        typedef typename Geom_traits::Rp::Intersect_3                                         Intersect_3;

        Compute_squared_distance compute_squared_distance = pct.geom_traits().compute_squared_distance_3_object();
        Construct_segment        construct_segment        = pct.geom_traits().construct_segment_3_object();
        typedef typename Constrained_triangulation_plus_2<CDT>::Vertices_in_constraint_iterator Vertices_in_constraint_iterator;

        // Get the previous and next vertices in the current simplified polyline
        Vertices_in_constraint_iterator vicp = boost::prior(vicq);
        Vertices_in_constraint_iterator vicr = boost::next(vicq);

        Point const& lP = (*vicp)->point();
        Point const& lR = (*vicr)->point();
        Point const& Q = (*vicq)->point();

        // Check if the segments (P,Q) or (Q,R) are too large when projected to the XY plane
        Point const& lP2 = Point(lP.x(), lP.y(), FT(0.0));
        Point const& lR2 = Point(lR.x(), lR.y(), FT(0.0));
        Point const& Q2 = Point(Q.x(), Q.y(), FT(0.0));
        if ( compute_squared_distance(lP2, Q2) > m_maxSqLength || compute_squared_distance(lR2, Q2) > m_maxSqLength ) {
            return std::numeric_limits<double>::infinity();
        }

        // Otherwise, check the cost
        Segment lP_R = construct_segment(lP, lR) ;

        // Using 3D distance
        //    FT d1 = 0.0;
        //    Points_in_constraint_iterator pp(vicp), rr(vicr);
        //    ++pp;
        //
        //    for ( ;pp != rr; ++pp )
        //      d1 = (std::max)(d1, compute_squared_distance( lP_R, *pp ) ) ;

        // Using distance in height
        Vector_3 cpLineAndZAxis = CGAL::cross_product<typename Geom_traits::Rp>(lP_R.to_vector(), Vector_3(0,0,1));
        Vector_3 planeNml = CGAL::cross_product<typename Geom_traits::Rp>(lP_R.to_vector(), cpLineAndZAxis);
        Plane_3 lineAsAPlane = Plane_3(lP, planeNml);

        FT d1 = 0.0;
        Points_in_constraint_iterator pp(vicp), rr(vicr);
        ++pp;
        for ( ;pp != rr; ++pp ) {
            // Construct a line in the Z direction
            Line_3 l(*pp, Vector_3(0, 0, 1));

            // Compute their intersection
            typedef typename CGAL::cpp11::result_of<typename Geom_traits::Rp::Intersect_3(Line_3, Plane_3)>::type IntersectionResult;
            IntersectionResult intersect = CGAL::intersection(l, lineAsAPlane);

            // If everything goes as expected, the intersection should exist and should be a point
            if (!intersect) {
                std::cerr << "Error! Empty intersection" << std::endl;
                return std::numeric_limits<double>::infinity();
            }
            if (const Line_3 *s = boost::get<Line_3>(&*intersect)) {
                std::cerr << "Error! Line intersection" << std::endl;
                return std::numeric_limits<double>::infinity();
            }

            // Get the intersection point
            const Point *ip = boost::get<Point>(&*intersect);

            // Finally, compute the squared distance between the query point and the intersection
            FT sqDist = CGAL::squared_distance<typename Geom_traits::Rp>(*pp, *ip);
            d1 = (std::max)(d1, sqDist) ;
        }

        return d1 ;
    }

private:
    // --- Attributes ---
    double m_maxSqLength;

};

} // namespace Polyline_simplification_2

} //namespace CGAL

#endif //EMODNET_QMGC_POINT_SET_FEATURES_SIMPLIFICATION_COST_H
