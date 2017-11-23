//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_CGAL_COUNT_AND_COST_STOP_PREDICATE_H_H
#define EMODNET_TOOLS_CGAL_COUNT_AND_COST_STOP_PREDICATE_H_H

#include <CGAL/Surface_mesh_simplification/Detail/Common.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_profile.h>
#include "cgal_defines.h"
#include <iostream>



namespace CGAL {

namespace Surface_mesh_simplification
{

/**
 * Stop predicate for the surface mesh simplification process.
 * Stops when the cost of the next edge to process is above a threshold, while ensuring a minimum number of edges left
 * after simplification.
 */
template<class ECM_>
class Cost_and_count_stop_predicate
{
public:

  typedef ECM_ ECM ;

  //  typedef Edge_profile<ECM> Profile ;

  typedef typename boost::graph_traits<ECM>::edges_size_type size_type ;

//  typedef typename K::FT FT ;

public :

  Cost_and_count_stop_predicate( FT costThres, std::size_t countThres )
          : m_costThres(costThres), m_countThres(countThres) {}

  template <typename F, typename Profile>
  bool operator()( F const&          aCurrentCost
                 , Profile const& // aEdgeProfile
                 , std::size_t    // aInitialCount
                 , std::size_t       aCurrentCount
                 ) const
  {
      //std::cout << "current cost = " << aCurrentCost << std::endl ;
//      std::cout << "current count = " << aCurrentCount << std::endl ;
    return (aCurrentCost > m_costThres) || ( aCurrentCount < m_countThres ) ;
  }

private:

  std::size_t m_countThres ;
  FT m_costThres ;
};

} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif //EMODNET_TOOLS_CGAL_COUNT_AND_COST_STOP_PREDICATE_H_H
