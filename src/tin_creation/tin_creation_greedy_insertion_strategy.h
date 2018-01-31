//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#ifndef EMODNET_TOOLS_TIN_CREATION_GREEDY_INSERTION_H
#define EMODNET_TOOLS_TIN_CREATION_GREEDY_INSERTION_H

#include "tin_creator.h"
#include "tin_creation_cgal_types.h"
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <memory>
#include <boost/heap/fibonacci_heap.hpp>

namespace TinCreation {

/**
 * @class HeapNode
 * @brief The information to be maintained in the heap structure
 */
struct GIHeapNode {
    FT error ;
    Point_3 candidate ;

    GIHeapNode() : error(0.0), candidate(Point_3(.0,.0,.0)) {}
    GIHeapNode( const FT& e, const Point_3& c ) : error(e), candidate(c) {}
};

struct CompareGIHeapNodes
{
    bool operator()(const GIHeapNode& n1, const GIHeapNode& n2) const
    {
        return n1.error < n2.error;
    }
};

typedef boost::heap::fibonacci_heap<GIHeapNode, boost::heap::compare<CompareGIHeapNodes>> GIHeap ;
typedef typename GIHeap::handle_type GIHeapNodeHandle ;

/// Additional information associated to a face in a Delaunay triangulation required by the Greedy Insertion



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


class TinCreationGreedyInsertionStrategy : public TinCreationStrategy
{
public:
    // --- Typedefs ---
    typedef CGAL::Triangulation_vertex_base_2<Gt>                       Vb;
    typedef CGAL::Triangulation_face_base_with_info_2<GIFaceInfo, Gt>   Fb;
//    typedef GIFaceBase<Gt>                                              Fb;
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
    enum ErrorType { ErrorHeight=0, Error3D } ;

    // --- Public Methods ---
    TinCreationGreedyInsertionStrategy( double approxTolerance,
                                        int initGridSamples = -1,
                                        int errorType = ErrorHeight )
            : m_sqApproxTol(approxTolerance*approxTolerance)
            , m_initGridSamples(initGridSamples)
            , m_errorType(errorType) {}

    Polyhedron create( const std::vector<Point_3>& dataPts,
                       const bool& constrainEasternVertices,
                       const bool& constrainWesternVertices,
                       const bool& constrainNorthernVertices,
                       const bool& constrainSouthernVertices ) ;
private:
    // --- Attributes ---
    FT m_sqApproxTol ;
    std::vector<Point_3> m_dataPts ;
    DT m_dt ;
    GIHeap m_heap;
    int m_errorType;
    int m_initGridSamples;

    // --- Private Methods ---
    /// Initialize the data structures
    void initialize(const bool& constrainEasternVertices,
                    const bool& constrainWesternVertices,
                    const bool& constrainNorthernVertices,
                    const bool& constrainSouthernVertices) ;

    /// Compute the error induced by a point w.r.t. a triangle
    /// \pre the triangle \param t is the closest to point p in the current approximation
    FT error(const Point_3& p, const Triangle_3&t ) const ;

    /// Compute the error given a point/triangle
    /// \pre Point XY projection falls within triangle XY projection
    FT errorHeight(const Point_3& p, const Triangle_3&t ) const ;

    /// Compute the error given a point/triangle. Orthogonal distance.
    /// \pre Point XY projection falls within triangle XY projection
    FT error3D(const Point_3& p, const Triangle_3&t ) const ;

    FT eval(const Point_3& p, const Triangle_3&t) const;

    /// Contains all the steps to perform when inserting a new point in the triangulation
    void insert( const Point_3& p ) ;

    /**
     * Find the point falling in the face inducing the largest error and add it to the heap.
     * Also adds the reference to the introduced heap node in the face.
     * \pre The face has its internal points ptrs set using the findPointsInFace function
     */
    void computeErrorAndUpdateHeap( FaceHandle fh ) ;
};

} // End namespace TinCreation

#endif //EMODNET_TOOLS_TIN_CREATION_GREEDY_INSERTION_H
