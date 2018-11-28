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

#ifndef EMODNET_QMGC_POLYLINES_FROM_FEATURED_EDGES_H
#define EMODNET_QMGC_POLYLINES_FROM_FEATURED_EDGES_H

#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/properties_Surface_mesh.h>
#include <boost/graph/filtered_graph.hpp>
#include <CGAL/boost/graph/split_graph_into_polylines.h>


/**
 * @brief Struct/functor that returns true if the edge is sharp (given a previously computed edgeIsSharp map)
 * @tparam Surface_mesh
 * @tparam EdgeIsSharpMap
 */
template<typename Surface_mesh, typename EdgeIsSharpMap>
struct IsSharpEdge {
    typedef typename boost::graph_traits<Surface_mesh>::edge_descriptor edge_descriptor;

    IsSharpEdge() {} // required by boost::filtered_graph

    IsSharpEdge(EdgeIsSharpMap eisMap) : m_eisMap(eisMap) {}

    bool operator()(typename boost::graph_traits<Surface_mesh>::edge_descriptor e) const {
        return get(m_eisMap, e);
    }

    EdgeIsSharpMap m_eisMap;
};

/**
 * @brief Visitor used by the extract_polylines_from_sharp_edges function
 * @tparam Graph
 * @tparam Polyline
 */
template<typename Graph,
         typename Polyline>
struct ExtractPolylinesVisitor
{
    std::vector<Polyline>& m_polylines;
    const Graph& m_graph;

    ExtractPolylinesVisitor(const Graph& graph, typename std::vector<Polyline>& polylines)
            : m_polylines(polylines), m_graph(graph)
    {}

    void start_new_polyline()
    {
        m_polylines.push_back(Polyline());
    }

    void add_node(typename boost::graph_traits<Graph>::vertex_descriptor vd)
    {
        if(m_polylines.back().empty()) {
            m_polylines.back().push_back(m_graph[vd]);
        }
    }

    void add_edge(typename boost::graph_traits<Graph>::edge_descriptor ed)
    {
        typename boost::graph_traits<Graph>::vertex_descriptor
                s = source(ed, m_graph),
                t = target(ed, m_graph);
        Polyline& polyline = m_polylines.back();
        CGAL_assertion(!polyline.empty());
        if(polyline.back() != m_graph[s]) {
            polyline.push_back(m_graph[s]);
        } else if(polyline.back() != m_graph[t]) {
            // if the edge is zero-length, it is ignored
            polyline.push_back(m_graph[t]);
        }
    }

    void end_polyline()
    {
        // ignore degenerated polylines
        if(m_polylines.back().size() < 2)
            m_polylines.resize(m_polylines.size() - 1);
    }
};

/**
 * @brief Extracts polylines from a set of sharp edges in a surface mesh
 * @tparam Surface_mesh Surface mesh type
 * @tparam EdgeIsSharpMap EdgeIsSharpMap type
 * @tparam Polyline Polyline type
 * @param sm The input surface mesh
 * @param eisMap Map indicating wether an edge in the surface mesh is sharp or not
 * @param polylines The set of polylines extracted
 */
template <typename Surface_mesh, typename EdgeIsSharpMap, typename Polyline>
void extract_polylines_from_sharp_edges(const Surface_mesh& sm,
                                        const EdgeIsSharpMap& eisMap,
                                        std::vector<Polyline>& polylines)
{
    typedef typename boost::property_traits<typename boost::property_map<Surface_mesh,CGAL::vertex_point_t>::type>::value_type Point_3;
    typedef boost::adjacency_list<
            boost::setS, // this avoids parallel edges
            boost::vecS,
            boost::undirectedS,
            Point_3 > Featured_edges_copy_graph;
    typedef IsSharpEdge<Surface_mesh, EdgeIsSharpMap> IsSharpEdge;

    // Filter the "graph" (i.e., the mesh) using the created property map
    IsSharpEdge isSharpEdge(eisMap);
    typedef boost::filtered_graph<Surface_mesh, IsSharpEdge > FilteredGraph;
    FilteredGraph filteredGraph(sm, isSharpEdge);

    // Create a copy of the graph containing just the featured edges
    Featured_edges_copy_graph feGraph;
    typedef typename boost::property_map<Surface_mesh, CGAL::vertex_point_t>::const_type Vpm;
    Vpm vpm = get(CGAL::vertex_point, sm);

    typedef std::map<Point_3, typename boost::graph_traits<Featured_edges_copy_graph>::vertex_descriptor> P2vmap;
    P2vmap p2vmap;

    // --> Add vertices
    BOOST_FOREACH(typename FilteredGraph::vertex_descriptor v, vertices(filteredGraph)){
                    typename Featured_edges_copy_graph::vertex_descriptor vc;
                    typename P2vmap::iterator it = p2vmap.find(get(vpm,v));
                    if(it == p2vmap.end()) {
                        vc = add_vertex(feGraph);
                        feGraph[vc] = get(vpm, v);
                        p2vmap[get(vpm,v)] = vc;
                    }
                }

    // --> Add edges
    BOOST_FOREACH(typename FilteredGraph::edge_descriptor e, edges(filteredGraph)) {
                    typename Featured_edges_copy_graph::vertex_descriptor vs = p2vmap[get(vpm, source(e, filteredGraph))];
                    typename Featured_edges_copy_graph::vertex_descriptor vt = p2vmap[get(vpm, target(e, filteredGraph))];
                    CGAL_warning_msg(vs != vt, "ignore self loop");
                    if (vs != vt) {
                        const std::pair<typename Featured_edges_copy_graph::edge_descriptor, bool> pair = add_edge(vs, vt, feGraph);
                    }
                }

    // Create the polylines
    ExtractPolylinesVisitor<Featured_edges_copy_graph, Polyline> visitor(feGraph, polylines);
    CGAL::split_graph_into_polylines(feGraph, visitor);
}


#endif //EMODNET_QMGC_POLYLINES_FROM_FEATURED_EDGES_H
