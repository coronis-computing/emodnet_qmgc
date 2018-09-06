//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_CGAL_SQUARED_DISTANCE_3_COST_H
#define EMODNET_TOOLS_CGAL_SQUARED_DISTANCE_3_COST_H

#include <CGAL/algorithm.h>

namespace CGAL {

template < class Tr >
class Constrained_triangulation_plus_2;

namespace Polyline_simplification_2
{

/// This class is a cost function which calculates the cost as the square of the distance between the original and simplified polylines in R3. To be used with Projection_traits_xy_3 class.
class Squared_distance_3_cost
{

public:

  /// Initializes the cost function
  Squared_distance_3_cost(const double& maxLength) : m_maxSqLength(maxLength*maxLength) {}

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
    typedef typename Constrained_triangulation_plus_2<CDT>::Points_in_constraint_iterator Points_in_constraint_iterator;
    typedef typename Constrained_triangulation_plus_2<CDT>::Geom_traits Geom_traits ;
    typedef typename Geom_traits::FT                                  FT;
    typedef typename Geom_traits::Compute_squared_distance_3 Compute_squared_distance;
    typedef typename Geom_traits::Compute_squared_distance_2 Compute_squared_distance_2;
    typedef typename Geom_traits::Construct_segment_3        Construct_segment;
    typedef typename Geom_traits::Segment_3                  Segment;
    typedef typename Geom_traits::Point_3                    Point;
    typedef typename Geom_traits::Point_2                    Point_2;

    Compute_squared_distance compute_squared_distance = pct.geom_traits().compute_squared_distance_3_object();
    Compute_squared_distance_2 compute_squared_distance_2 = pct.geom_traits().compute_squared_distance_2_object();
    Construct_segment        construct_segment        = pct.geom_traits().construct_segment_3_object();
    typedef typename Constrained_triangulation_plus_2<CDT>::Vertices_in_constraint_iterator Vertices_in_constraint_iterator;

    Vertices_in_constraint_iterator vicp = boost::prior(vicq);
    Vertices_in_constraint_iterator vicr = boost::next(vicq);

    Point const& lP = (*vicp)->point();
    Point const& lR = (*vicr)->point();
    Point const& Q = (*vicq)->point();

    // Check if the segments (P,Q) or (Q,R) are too large when projected to the XY plane
    Point const& lP2 = Point((*vicp)->point().x(), (*vicp)->point().y(), FT(0.0));
    Point const& lR2 = Point((*vicr)->point().x(), (*vicr)->point().y(), FT(0.0));
    Point const& Q2 = Point((*vicq)->point().x(), (*vicq)->point().y(), FT(0.0));
    if ( compute_squared_distance(lP2, Q2) > m_maxSqLength || compute_squared_distance(lR2, Q2) > m_maxSqLength ) {
        return std::numeric_limits<double>::infinity();
    }


    // Otherwise, check the cost using 3D distance
    Segment lP_R = construct_segment(lP, lR) ;

    FT d1 = 0.0;
    Points_in_constraint_iterator pp(vicp), rr(vicr);
    ++pp;

    for ( ;pp != rr; ++pp )
      d1 = (std::max)(d1, compute_squared_distance( lP_R, *pp ) ) ;

    return d1 ;
  }

private:
    // --- Attributes ---
    double m_maxSqLength;

};


} // namespace Polyline_simplification_2


} //namespace CGAL

#endif // End namespace EMODNET_TOOLS_CGAL_SQUARED_DISTANCE_3_COST_H
