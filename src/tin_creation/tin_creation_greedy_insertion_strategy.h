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

#ifndef EMODNET_QMGC_TIN_CREATION_GREEDY_INSERTION_H
#define EMODNET_QMGC_TIN_CREATION_GREEDY_INSERTION_H

#include "tin_creator.h"
#include "tin_creation_cgal_types.h"
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <memory>
#include <boost/heap/fibonacci_heap.hpp>
#include "tin_creation_utils.h"

namespace TinCreation {

/**
 * @class GIHeapNode
 * @brief The information to be maintained in the heap structure
 */
struct GIHeapNode {
    FT error ;
    Point_3 candidate ;

    GIHeapNode() : error(0.0), candidate(Point_3(.0,.0,.0)) {}
    GIHeapNode( const FT& e, const Point_3& c ) : error(e), candidate(c) {}
};

/**
 * @class CompareGIHeapNodes
 * @brief Comparison operator (less than) for GIHeapNodes
 */
struct CompareGIHeapNodes
{
    bool operator()(const GIHeapNode& n1, const GIHeapNode& n2) const
    {
        return n1.error < n2.error;
    }
};

typedef boost::heap::fibonacci_heap<GIHeapNode, boost::heap::compare<CompareGIHeapNodes>> GIHeap ;
typedef typename GIHeap::handle_type GIHeapNodeHandle ;

/**
 * @class GIFaceInfo
 * @brief Additional information associated to a face in a Delaunay triangulation required by the Greedy Insertion algorithm
 */
class GIFaceInfo {
public:
    GIFaceInfo() : m_pointsInFacePtrs(), m_heapNodeHandle(), m_hasHeapNodeHandle(false) {}
    ~GIFaceInfo() {}

    void setHeapNodeHandle( GIHeapNodeHandle h ) { m_heapNodeHandle = h; m_hasHeapNodeHandle = true ; }
    void addPointPtr(std::shared_ptr<Point_3> pp) {m_pointsInFacePtrs.push_back(pp);}
    std::vector<std::shared_ptr<Point_3>> getPtsSharedPtrs() { return m_pointsInFacePtrs; } ;
    size_t getNumPtsInFace() { return m_pointsInFacePtrs.size(); }
    bool hasHeapNodeHandle() {return m_hasHeapNodeHandle; }
    GIHeapNodeHandle getHeapNodeHandle() { return m_heapNodeHandle ; }

    // Some face handles may not be erased after insertion, apply this to all the faces on the conflict zone prior to insert a point to ensure that all the faces' info are empty
    void clearInfo() {
        m_pointsInFacePtrs.clear();
        m_heapNodeHandle = GIHeapNodeHandle();
        m_hasHeapNodeHandle = false;
    };
private:
    std::vector<std::shared_ptr<Point_3>> m_pointsInFacePtrs;
    GIHeapNodeHandle m_heapNodeHandle ;
    bool m_hasHeapNodeHandle ;
};

/**
 * @class TinCreationGreedyInsertionStrategy
 * @brief Creates a TIN using the Greedy Insertion method.
 *
 * This class implements a slighly modified version of the method described in:
 *
 * M. Garland, P. S. Heckbert, Fast polygonal approximation of terrains and height Fields, Technical report CMU-CS-95-181, Carnegie Mellon University 575 (September 1995).
 */
class TinCreationGreedyInsertionStrategy : public TinCreationStrategy
{
public:
    // --- Typedefs ---
    typedef CGAL::Triangulation_vertex_base_2<Gt>                       Vb;
    typedef CGAL::Triangulation_face_base_with_info_2<GIFaceInfo, Gt>   Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                Tds;
    typedef CGAL::Delaunay_triangulation_2<Gt, Tds>                     DT;
    typedef DT::Face_handle                                             FaceHandle;
    typedef DT::Vertex_handle                                           VertexHandle;
    typedef DT::Face_circulator                                         FaceCirculator;
    typedef Gt::Rp::Triangle_3                                          Triangle_3;
    typedef Gt::Rp::Vector_3                                            Vector_3;
    typedef Gt::Rp::Segment_3                                           Segment_3;
    typedef Gt::Rp::Line_3                                              Line_3;
    typedef Gt::FT                                                      FT;
    typedef Gt::Rp::Intersect_3                                         Intersect_3;

public:
    enum ErrorType { ErrorHeight=0, Error3D } ; //! Types of errors to use

    // --- Public Methods ---

    /**
     * Constructor
     * @param rootApproxTolerance Approximation tolerance (the error guiding the end of the process)
     * @param initGridSamples Size of the fixed grid of samples to use. It effectively defines a minimum complexity on the mesh.
     * @param errorType Type of error to use.
     */
    TinCreationGreedyInsertionStrategy(double rootApproxTolerance,
                                       int initGridSamples = -1,
                                       int errorType = ErrorHeight)
            : m_initGridSamples(initGridSamples)
            , m_errorType(errorType)
    {
        m_approxTolPerZoom = std::vector<FT>{rootApproxTolerance};
        setParamsForZoom(0);
    }

    /**
     * Constructor
     * @param rootApproxTolerance Approximation tolerances per zoom (the errors guiding the end of the process)
     * @param initGridSamples Size of the fixed grid of samples to use. It effectively defines a minimum complexity on the mesh.
     * @param errorType Type of error to use.
     */
    TinCreationGreedyInsertionStrategy(const std::vector<FT>& approxTolPerZoom,
                                       int initGridSamples = -1,
                                       int errorType = ErrorHeight)
            : m_initGridSamples(initGridSamples)
            , m_errorType(errorType)
            , m_approxTolPerZoom(approxTolPerZoom)
    {
        setParamsForZoom(0);
    }

    void setParamsForZoom(const unsigned int& zoom)
    {
        m_approxTol = standardHandlingOfThresholdPerZoom(m_approxTolPerZoom, zoom);
    }

    Polyhedron create(const std::vector<Point_3>& dataPts,
                      const bool& constrainEasternVertices,
                      const bool& constrainWesternVertices,
                      const bool& constrainNorthernVertices,
                      const bool& constrainSouthernVertices);
private:
    // --- Attributes ---
    FT m_approxTol ;
    FT m_scaledSqApproxTol ;
    std::vector<Point_3> m_dataPts ;
    DT m_dt ;
    GIHeap m_heap;
    int m_errorType;
    int m_initGridSamples;
    std::vector<FT> m_approxTolPerZoom; // in metric, not squared!

    // --- Private Methods ---
    /// Initialize the data structures
    void initialize(const bool& constrainEasternVertices,
                    const bool& constrainWesternVertices,
                    const bool& constrainNorthernVertices,
                    const bool& constrainSouthernVertices);

    /// Compute the error induced by a point w.r.t. a triangle
    /// \pre the triangle \param t is the closest to point p in the current approximation
    FT error(const Point_3& p, const Triangle_3&t) const;

    /// Compute the error given a point/triangle
    /// \pre Point's XY projection falls within triangle's XY projection
    FT errorHeight(const Point_3& p, const Triangle_3&t) const;

    /// Compute the error given a point/triangle. Orthogonal distance.
    /// \pre Point's XY projection falls within triangle's XY projection
    FT error3D(const Point_3& p, const Triangle_3&t) const;

    /// Evaluate the height of a point in the current approximation. Just the XY part of \p p is used.
    /// \pre Point's XY projection falls within triangle's XY projection
    FT eval(const Point_3& p, const Triangle_3&t) const;

    /// Contains all the steps to perform when inserting a new point in the triangulation
    void insert(const Point_3& p);

    /**
     * Find the point falling in the face inducing the largest error and add it to the heap.
     * Also adds the reference to the introduced heap node in the face.
     * \pre The face has its internal points ptrs set using the findPointsInFace function
     */
    void computeErrorAndUpdateHeap(FaceHandle fh);
};

} // End namespace TinCreation

#endif //EMODNET_QMGC_TIN_CREATION_GREEDY_INSERTION_H
