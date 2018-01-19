//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_greedy_insertion_strategy.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/ch_selected_extreme_points_2.h>
#include <algorithm>
#include <CGAL/intersections.h>
#include "cgal_utils.h"



Polyhedron TinCreationGreedyInsertionStrategy::create( const std::vector<Point_3>& dataPts,
                                               const bool& constrainEasternVertices,
                                               const bool& constrainWesternVertices,
                                               const bool& constrainNorthernVertices,
                                               const bool& constrainSouthernVertices )
{
    m_dataPts = dataPts ; // Copy the data points
    m_dt.clear() ; // To clear data from other executions using this same object
    m_heap.clear() ; // To clear data from other executions using this same object

    // Initialize the data structures
    initialize() ;

    // Try to perform one step
    while (!m_heap.empty()) {
        // Pop the first element in the priority heap
        GIHeapNode nh = m_heap.top();
        // WARNING: do NOT pop the heap's top entry here, it will be effectively done in the insert function

//        std::cout << "Points in triangulation: " << std::endl;
//        for ( DT::Vertex_iterator it = m_dt.finite_vertices_begin(); it != m_dt.finite_vertices_end(); ++it ) {
//            std::cout << it->point() << std::endl ;
//        }

        // Insert the point and update the internal structures
//        std::cout << "Processing candidate = " << nh.candidate << std::endl ;
        insert(nh.candidate) ;

    }

//    std::cout << "Greedy insertion finished" << std::endl ;

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<DT, HalfedgeDS> builderDT(m_dt);
    surface.delegate(builderDT);

    return surface ;
}


void TinCreationGreedyInsertionStrategy::initialize()
{
    // Create the base mesh as the two triangles covering the tile
    std::vector<Point_3> chPts ;
    CGAL::convex_hull_2( m_dataPts.begin(), m_dataPts.end(), std::back_inserter(chPts), CGAL::Projection_traits_xy_3<K>() );

    // Insert them in the triangulation
    m_dt.insert(chPts.begin(), chPts.end()) ;

    // Remove corner points from the data points to be considered
    for (std::vector<Point_3>::iterator it = chPts.begin(); it != chPts.end(); ++it) {
        m_dataPts.erase( std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end() ) ;
    }
    // NOTE: From now on, the vector m_dataPts should not be modified again, as the GIFaceInfo class will maintain a list of pointers to this vector!

//    std::vector<Point_3>::iterator itCorner0, itCorner1, itCorner2, itCorner3 ;
//    CGAL::ch_nswe_point(m_dataPts.begin(), m_dataPts.end(), itCorner0, itCorner1, itCorner2, itCorner3, CGAL::Projection_traits_xy_3<K>() );
//
//    std::cout << "Extremal points" << std::endl;
//    std::cout << "corner 0 = " << *itCorner0 << std::endl ;
//    std::cout << "corner 1 = " << *itCorner1 << std::endl ;
//    std::cout << "corner 2 = " << *itCorner2 << std::endl ;
//    std::cout << "corner 3 = " << *itCorner3 << std::endl ;

    // For all the points in the data set, check in which triangle they fall
    for ( std::vector<Point_3>::iterator it = m_dataPts.begin(); it != m_dataPts.end(); ++it )
    {
        // Locate the triangle containing the current point
        FaceHandle fh = m_dt.locate(*it) ;

        // Add the point to the list of points in this triangle
//        fh->addPointPtr(std::shared_ptr<Point_3>( &(*it) ));
        int ind = it - m_dataPts.begin() ;

        fh->addPointInd(ind);
    }

    // Select the best candidate for each face
    DT::Finite_faces_iterator fit;
    for (fit = m_dt.finite_faces_begin(); fit != m_dt.finite_faces_end(); fit++)
    {
        computeErrorAndUpdateHeap(fit) ;
    }
}


FT TinCreationGreedyInsertionStrategy::error(const Point_3& p, const Triangle_3&t )
{
    // Construct a line in the Z direction
    Line_3 l( p, Vector_3(0,0,1) );

    // Compute their intersection
    CGAL::cpp11::result_of<Intersect_3(Line_3, Triangle_3)>::type intersect = CGAL::intersection(l, t);

    // If everything goes as expected, the intersection should exist and should be a point
    if (!intersect) {
        std::cerr << "Error! Empty intersection" << std::endl;
        return FT(0.0);
    }
    if (const Segment_3* s = boost::get<Segment_3>(&*intersect)) {
        std::cerr << "Error! Segment intersection" << std::endl;
        return FT(0.0);
    }

    // Get the intersection point
    const Point_3* ip = boost::get<Point_3 >(&*intersect);

    // Finally, compute the squared distance between the query point and the intersection
    return CGAL::squared_distance(p, *ip) ;
}



FT TinCreationGreedyInsertionStrategy::error3D(const Point_3& p, const Triangle_3&t )
{
    return CGAL::squared_distance(p, t.supporting_plane()) ;
}



void
TinCreationGreedyInsertionStrategy::
insert(const Point_3& p)
{
    // Get the conflict zone in the Delaunay triangulation for the point to be inserted
    std::vector<FaceHandle> facesInConflict;
    m_dt.get_conflicts( p, std::back_inserter(facesInConflict) ) ;

    // All the faces in the conflict zone will be changed
    // Thus, collect all the points falling in these faces and delete their corresponding entries in the heap
//    std::vector<std::shared_ptr<Point_3>> ptsInConflictZone ;
    std::vector<int> ptsIndInConflictZone ;
    std::vector<FaceHandle>::iterator fit ;
//    std::cout << "facesInConflict = " << facesInConflict.size() << std::endl ;
    for (fit = facesInConflict.begin(); fit != facesInConflict.end(); fit++) {
//        std::cout << "Num pts in face = " << (*(*fit)).getNumPtsInFace() << std::endl ;

        // Collect points' pointers in this face
//        std::vector<std::shared_ptr<Point_3>> ptsPtrs = (*(*fit)).getPtsSharedPtrs() ;
//        std::vector<Point_3*> ptsPtrs = (*(*fit)).getPtsSharedPtrs() ;
        std::vector<int> ptsInd = (*(*fit)).getPtsInds() ;
//        std::cout << "Num pts inds in face = " << ptsInd.size() << std::endl ;
        for (int i = 0; i < ptsInd.size(); i++)
            ptsIndInConflictZone.push_back(ptsInd[i]);

//        std::cout << "Num pts inds in conflict zone = " << ptsIndInConflictZone.size() << std::endl ;

        // Delete heap entry associated to the face, if any
//        std::unique_ptr<GIHeapNodeHandle> hh = (*(*fit)).getHeapHandleUniquePtr() ;
        if ((*fit)->hasHeapNodeHandle()) {
//            std::cout << "Has node handle" << std::endl ;
            m_heap.erase((*(*fit)).getHeapNodeHandle());
        }

        // Some face handles may not be erased after insertion, clear info as it will be recalculated for the new face shape in case it is not eliminated by insertion
        (*(*fit)).clearInfo();
    }
//    std::cout << "Num pts on conflict zone = " << ptsIndInConflictZone.size() << std::endl ;

//    std::cout << "Inserting the point and modifying the triangulation" << std::endl ;

    // Insert the point and modify the triangulation
    VertexHandle vh = m_dt.insert(p) ; // TODO: Use point location once to get a nearby face and then use this face as guess?

//    std::cout << "Checking point locations" << std::endl ;

    // Check in which faces previously collected points fall
    // While there is no check in this regard, they are all supposed to be falling in the newly created faces that are now
    // covering the previous conflict zone.
//    std::vector<std::shared_ptr<Point_3>>::iterator it ;
    std::vector<int>::iterator itInd ;
    for (itInd = ptsIndInConflictZone.begin(); itInd != ptsIndInConflictZone.end(); ++itInd) {
        Point_3 p = m_dataPts[*itInd] ;

//        std::cout << "Locating point = " << p << std::endl ;

        // Locate the triangle containing the current point
        FaceHandle fh = m_dt.locate(p);

//        std::cout << "Num pts in face = " << fh->getNumPtsInFace() << std::endl ;

//        std::cout << "Adding point" << std::endl ;
        // Add the point to the
        // list of points in this triangle
        fh->addPointInd( *itInd );
    }

//    std::cout << "DONE" << std::endl ;

    // Update the internal structure of the new faces with the points falling on them, and also update the heap
    FaceCirculator fc = m_dt.incident_faces(vh), end(fc) ;
    do{
        if (!m_dt.is_infinite(fc)) {
//            std::cout << "getNumPtsInFace = " << fc->getNumPtsInFace() << std::endl ;

            // Compute the maximum error on this face and update the heap
            computeErrorAndUpdateHeap(fc);
        }
    } while ( ++fc != end );
}



void
TinCreationGreedyInsertionStrategy::
computeErrorAndUpdateHeap( FaceHandle fh )
{
    Triangle_3 t = m_dt.triangle(fh) ;

    // Run over candidates and find the one inducing the worst error
    Point_3 best;
    FT maxSqError = m_sqApproxTol ;

    //std::vector<std::shared_ptr<Point_3>> ptsPtrs = fh->getPtsSharedPtrs() ;
//    std::vector<std::shared_ptr<Point_3>>::iterator itPtPtr ;
    //std::vector<Point_3*> ptsPtrs = fh->getPtsSharedPtrs() ;
    std::vector<int> ptsInds = fh->getPtsInds() ;
    std::vector<int>::iterator itPtInd ;
    for (itPtInd = ptsInds.begin();
         itPtInd != ptsInds.end(); ++itPtInd)
    {
        Point_3 p = Point_3(m_dataPts[*itPtInd]);
        FT e = error3D(p, t) ;
        if (e > maxSqError) {
            best = p ;
            maxSqError = e ;
        }
    }
    if ( maxSqError > m_sqApproxTol ) {
//        std::cout << "maxSqError = " << maxSqError << std::endl ;
//        std::cout << "candidate = " << best << std::endl ;
        GIHeapNodeHandle nh = m_heap.push(GIHeapNode(maxSqError, best));
        fh->setHeapNodeHandle( nh );
    }
//    else {
//        // All the points in this face have an error below the threshold.
//        // Thus, there is no candidate for this face, and the heap gets no update
////        std::cout << "NO candidate" << std::endl ;
//    }
}