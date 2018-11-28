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

#ifndef EMODNET_QMGC_POLYHEDRON_BUILDER_FROM_C3T3_BOUNDARY_H
#define EMODNET_QMGC_POLYHEDRON_BUILDER_FROM_C3T3_BOUNDARY_H

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include "tin_creation/tin_creation_cgal_types.h"
#include <CGAL/array.h>
#include <map>



/**
 * @class PolyhedronBuilderFromC3T3Boundary
 * @brief A modifier creating a Polyhedron_3 structure with the incremental builder from the boundary in a C3T3
 */
template<class C3T3, class HDS>
class PolyhedronBuilderFromC3T3Boundary : public CGAL::Modifier_base<HDS> {
public:
    C3T3 m_c3t3 ;
    typename C3T3::Subdomain_index  m_index ;
    typedef typename C3T3::Point::Point  Point_3;

    PolyhedronBuilderFromC3T3Boundary( const C3T3& c,
                                       const typename C3T3::Subdomain_index& index ) : m_c3t3(c), m_index(index) {}

    void operator()( HDS& hds )
    {
        // Run over the data needed to create the Polyhedron
        typedef typename C3T3::Triangulation Triangulation;
        typedef typename Triangulation::Vertex_handle Vertex_handle;

        // Vertex-->index map
        std::map<Vertex_handle, std::size_t> V;

        std::vector<Point_3 > vertices ;
        std::vector<std::vector<std::size_t>> facesIndices ;

        std::size_t numVerts = 0;
        std::size_t numFacets = 0;
        for(typename C3T3::Facets_in_complex_iterator
                    fit = m_c3t3.facets_in_complex_begin(),
                    end = m_c3t3.facets_in_complex_end();
            fit != end; ++fit)
        {
            typename C3T3::Subdomain_index cell_sd = m_c3t3.subdomain_index(fit->first);
            typename C3T3::Subdomain_index opp_sd = m_c3t3.subdomain_index(fit->first->neighbor(fit->second));

            if (cell_sd!=m_index && opp_sd!=m_index) continue;

            numFacets++;
            int j=0;

            std::vector<std::size_t> indices; indices.resize(3);
            for (int i = 0; i < 4; ++i) {
                if (i != fit->second) {
                    // Try to insert the point in the map
                    Vertex_handle v = (*fit).first->vertex(i) ;
                    std::pair<typename std::map<Vertex_handle, std::size_t>::iterator, bool> res = V.insert(std::make_pair(v,numVerts));

                    if (res.second){
                        // The vertex was not in the map, add it to the Polygon
                        numVerts++;
                        vertices.push_back( res.first->first->point().point() );
                    }
                    indices[j++] = res.first->second;
                }
            }
            facesIndices.push_back(indices);
        }

        // Coherently orient the faces of the polyhedron
        CGAL::Polygon_mesh_processing::orient_polygon_soup(vertices, facesIndices);

        // Polyhedron_3 incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);

        // Begin the surface construction
        B.begin_surface( vertices.size(), facesIndices.size() );

        for( typename std::vector<Point_3>::iterator it = vertices.begin(); it != vertices.end(); ++it )
        {
            B.add_vertex( *it );
        }

        for( std::vector<std::vector<std::size_t>>::iterator it = facesIndices.begin(); it != facesIndices.end(); ++it )
        {
//            if( B.test_facet(it->begin(), it->end()) == true ) {
//                std::cout << "Test passed: " << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] << std::endl;
//                std::cout << "adding: " << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] << std::endl;
////                std::cout << "3 " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;
//                B.begin_facet();
//                B.add_vertex_to_facet( (*it)[0] );
//                B.add_vertex_to_facet( (*it)[1] );
//                B.add_vertex_to_facet( (*it)[2] );
//                B.end_facet();
//            }
//            else {
//                std::cout << "Test failed: " << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] << std::endl;
//
//                std::vector<std::size_t> tri = {(*it)[0], (*it)[2], (*it)[1]};
//                if( B.test_facet(tri.begin(), tri.end()) == true ) {
//                    std::cout << "adding: " << (*it)[0] << ", " << (*it)[2] << ", " << (*it)[1] << std::endl;
////                    std::cout << "3 " << (*it)[0] << " " << (*it)[2] << " " << (*it)[1] << std::endl;
//                    B.begin_facet();
//                    B.add_vertex_to_facet((*it)[0]);
//                    B.add_vertex_to_facet((*it)[2]);
//                    B.add_vertex_to_facet((*it)[1]);
//                    B.end_facet();
//                }
//                else {
//                    std::cout << "Test failed again: " << (*it)[0] << ", " << (*it)[2] << ", " << (*it)[1] << std::endl;
//                    std::cout << "adding: " << (*it)[1] << ", " << (*it)[0] << ", " << (*it)[2] << std::endl;
////                    std::cout << "3 " << (*it)[1] << " " << (*it)[0] << " " << (*it)[2] << std::endl;
//                    B.begin_facet();
//                    B.add_vertex_to_facet((*it)[1]);
//                    B.add_vertex_to_facet((*it)[0]);
//                    B.add_vertex_to_facet((*it)[2]);
//                    B.end_facet();
//                }
//            }
            B.begin_facet();
            B.add_vertex_to_facet( (*it)[0] );
            B.add_vertex_to_facet( (*it)[1] );
            B.add_vertex_to_facet( (*it)[2] );
            B.end_facet();
        }

        // End the surface
        B.end_surface();
    }
};

#endif //EMODNET_QMGC_POLYHEDRON_BUILDER_FROM_C3T3_BOUNDARY_H
