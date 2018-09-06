//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_greedy_insertion_strategy.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/ch_selected_extreme_points_2.h>
#include <algorithm>
#include <CGAL/intersections.h>
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
#include "cgal/extract_tile_borders_from_polyhedron.h"

namespace TinCreation {

Polyhedron TinCreationGreedyInsertionStrategy::create(const std::vector<Point_3> &dataPts,
                                                      const bool &constrainEasternVertices,
                                                      const bool &constrainWesternVertices,
                                                      const bool &constrainNorthernVertices,
                                                      const bool &constrainSouthernVertices) {
    m_dt.clear(); // To clear data from other executions using this same object
    m_heap.clear(); // To clear data from other executions using this same object
    m_dataPts = dataPts; // Copy the data points

    // Scale the approximation threshold to the units of the tile!
    m_scaledSqApproxTol = m_approxTol*this->getScaleZ();
    m_scaledSqApproxTol *= m_scaledSqApproxTol; // Squared value to ease distance computations

//    std::cout << "m_scaledSqApproxTol = " << m_scaledSqApproxTol << std::endl;

//    std::cout << "Start m_heap.size() = " << m_heap.size() << std::endl ;
//    std::cout << "max sq error = " << m_sqApproxTol << std::endl ;
//    std::cout << "m_dataPts.size() = " << m_dataPts.size() << std::endl ;

    // Initialize the data structures
    initialize(constrainEasternVertices, constrainWesternVertices,
               constrainNorthernVertices, constrainSouthernVertices);

//    std::cout << "Init m_heap.size() = " << m_heap.size() << std::endl ;

    // Try to perform one step
    while (!m_heap.empty()) {
        // Pop the first element in the priority heap
        GIHeapNode nh = m_heap.top();
        // WARNING: do NOT pop the heap's top entry here, it will be effectively done in the insert function

//        std::cout << "m_heap.size() = " << m_heap.size() << std::endl;

        // Insert the point and update the internal structures
        insert(nh.candidate);
    }

//    std::cout << "Finished processing" << std::endl;

    // Translate to Polyhedron
    Polyhedron surface;
    PolyhedronBuilderFromProjectedTriangulation<DT, HalfedgeDS> builderDT(m_dt);
    surface.delegate(builderDT);

    return surface;
}


void TinCreationGreedyInsertionStrategy::initialize(const bool &constrainEasternVertices,
                                                    const bool &constrainWesternVertices,
                                                    const bool &constrainNorthernVertices,
                                                    const bool &constrainSouthernVertices) {
    if (!constrainEasternVertices && !constrainWesternVertices && !constrainNorthernVertices &&
        !constrainSouthernVertices && m_initGridSamples <= 0) {
        // Create the base mesh as the two triangles covering the tile
        std::vector<Point_3> chPts;
        CGAL::convex_hull_2(m_dataPts.begin(), m_dataPts.end(), std::back_inserter(chPts),
                            CGAL::Projection_traits_xy_3<K>());

//        m_screenOutputMutex.lock();
//        std::cout << "Convex hull points = " << chPts.size() << std::endl;
//        for (std::vector<Point_3>::iterator it = chPts.begin(); it!= chPts.end(); ++it ) {
//            std::cout << *it << std::endl;
//        }
//        m_screenOutputMutex.unlock();

        // Insert them in the triangulation
        m_dt.insert(chPts.begin(), chPts.end());

        // Remove corner points from the data points to be considered
//        std::cout << "m_dataPts.size() (before) = " << m_dataPts.size() << std::endl;
        for (std::vector<Point_3>::iterator it = chPts.begin(); it != chPts.end(); ++it) {
//            std::cout << "Erasing point = " << *it << std::endl;
            m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
        }
//        std::cout << "m_dataPts.size() (after) = " << m_dataPts.size() << std::endl;
    } else {
        // Temporal triangulation with all the points
        DT tmpDt;
        tmpDt.insert(m_dataPts.begin(), m_dataPts.end());
        // Build the polyhedron
        Polyhedron poly;
        PolyhedronBuilderFromProjectedTriangulation<DT, HalfedgeDS> builderDT(tmpDt);
        poly.delegate(builderDT);
        poly.normalize_border();
        // Extract border vertices
        std::vector<Point_3> easternBorderPts, westernBorderPts, northernBorderPts, southernBorderPts;
        Point_3 cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11;
        bool res = extractTileBordersFromPolyhedron<Polyhedron>(poly, easternBorderPts, westernBorderPts, northernBorderPts, southernBorderPts,
                                                                cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11);

        if (!res) {
            std::cout << "Not all 4 corners were detected!" << std::endl;
        }

        // For each constrained border, add the points to the internal triangulation and remove the points from the data points
        if (constrainEasternVertices) {
//            std::cout << "Eastern vertices" << std::endl ;
            m_dt.insert(easternBorderPts.begin(), easternBorderPts.end());
            for (std::vector<Point_3>::iterator it = easternBorderPts.begin(); it != easternBorderPts.end(); ++it) {
//                std::cout << *it << std::endl ;
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
            }
        }
        if (constrainWesternVertices) {
            m_dt.insert(westernBorderPts.begin(), westernBorderPts.end());
//            std::cout << "Western vertices" << std::endl ;
            for (std::vector<Point_3>::iterator it = westernBorderPts.begin(); it != westernBorderPts.end(); ++it) {
//                std::cout << *it << std::endl ;
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
            }
        }
        if (constrainSouthernVertices) {
            m_dt.insert(southernBorderPts.begin(), southernBorderPts.end());
//            std::cout << "Southern vertices" << std::endl ;
            for (std::vector<Point_3>::iterator it = southernBorderPts.begin();
                 it != southernBorderPts.end(); ++it) {
//                std::cout << *it << std::endl ;
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
            }
        }
        if (constrainNorthernVertices) {
            m_dt.insert(northernBorderPts.begin(), northernBorderPts.end());
//            std::cout << "Northern vertices" << std::endl;
            for (std::vector<Point_3>::iterator it = northernBorderPts.begin();
                 it != northernBorderPts.end(); ++it) {
//                std::cout << *it << std::endl ;
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
            }
        }

//        if (constrainEasternVertices) std::cout << "constrainEasternVertices" << std::endl;
//        if (constrainWesternVertices) std::cout << "constrainWesternVertices" << std::endl;
//        if (constrainSouthernVertices) std::cout << "constrainSouthernVertices" << std::endl;
//        if (constrainNorthernVertices) std::cout << "constrainNorthernVertices" << std::endl;
//
//        std::cout << "Constraints added as base points" << std::endl;

        // If required, create the base grid
        if (m_initGridSamples > 0) {
            // Compute the extents of the terrain/tile
            FT startX = cornerPoint00.x();
            FT endX = cornerPoint11.x();
            FT startY = cornerPoint00.y();
            FT endY = cornerPoint11.y();

//            std::cout << "startX = " << startX << std::endl;
//            std::cout << "endX = " << endX << std::endl;
//            std::cout << "startY = " << startY << std::endl;
//            std::cout << "endY = " << endY << std::endl;

            FT extX = endX - startX;
            FT extY = endY - startY;
            FT stepX = extX / (double)m_initGridSamples;
            FT stepY = extY / (double)m_initGridSamples;
            std::vector<Point_3> insertedPts;
            for (int i = 0; i <= m_initGridSamples; i++) {
                if ( (i == 0 && constrainWesternVertices ) ||
                     (i == m_initGridSamples && constrainEasternVertices ) )
                    continue ;
                for (int j = 0; j <= m_initGridSamples; j++) {
                    if ( (j == 0 && constrainSouthernVertices) ||
                         (j == m_initGridSamples && constrainNorthernVertices ) )
                        continue ;

                    // Dummy point
                    FT curX = startX + (i*stepX);
                    FT curY = startY + (j*stepY);
                    Point_3 p(curX, curY, 0.0);

//                    std::cout << "curX = " << curX << std::endl;
//                    std::cout << "curY = " << curY << std::endl;

                    // Detect in which triangle does the projection of this grid point falls
                    FaceHandle fh = tmpDt.locate(p);
                    if (tmpDt.is_infinite(fh))
                        std::cout << "Infinite facet!!!!!!!!!!" << std::endl;
                    Triangle_3 t = tmpDt.triangle(fh);

                    FT h = eval(p, t);

                    Point_3 pi(curX, curY, h);

//                    std::cout << "Inserted steiner point = " << pi << std::endl;

                    m_dt.insert(pi);

                    insertedPts.push_back(pi);
                }
            }

            // Remove the inserted points in case they are part of the input data points
//            std::cout << "m_dataPts.size() (before)= " << m_dataPts.size() << std::endl;
            for (std::vector<Point_3>::iterator it = insertedPts.begin(); it != insertedPts.end(); ++it) {
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), *it), m_dataPts.end());
            }
//            std::cout << "m_dataPts.size() = " << m_dataPts.size() << std::endl;

//            std::cout << "Triangulation vertices:" << std::endl;
//            for (DT::Vertex_iterator vit = m_dt.vertices_begin(); vit != m_dt.vertices_end(); ++vit)
//                std::cout << vit->point() << std::endl;

        }
        else {
            // Add the corner points if none of its adjacent borders is constrained
            if (!constrainWesternVertices && !constrainSouthernVertices) {
                m_dt.insert(cornerPoint00);
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), cornerPoint00), m_dataPts.end());
            }
            if (!constrainWesternVertices && !constrainNorthernVertices) {
                m_dt.insert(cornerPoint01);
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), cornerPoint01), m_dataPts.end());
            }
            if (!constrainNorthernVertices && !constrainEasternVertices) {
                m_dt.insert(cornerPoint11);
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), cornerPoint11), m_dataPts.end());
            }
            if (!constrainEasternVertices && !constrainSouthernVertices) {
                m_dt.insert(cornerPoint10);
                m_dataPts.erase(std::remove(m_dataPts.begin(), m_dataPts.end(), cornerPoint10), m_dataPts.end());
            }
        }
    }
    // NOTE: From now on, the vector m_dataPts should not be modified again, as the GIFaceInfo class will maintain a list of pointers to this vector!

    // For all the points in the data set, check in which triangle they fall
//    std::cout << "Evaluating the points falling on the initial faces" << std::endl;
    for (std::vector<Point_3>::iterator it = m_dataPts.begin(); it != m_dataPts.end(); ++it) {
        // Locate the triangle containing the current point
        FaceHandle fh = m_dt.locate(*it);

        // Add the point to the list of points in this triangle
        fh->info().addPointPtr(std::make_shared<Point_3>(*it));

//        std::cout << *it << std::endl;
    }

    // Select the best candidate for each face
    DT::Finite_faces_iterator fit;
    for (fit = m_dt.finite_faces_begin(); fit != m_dt.finite_faces_end(); fit++) {
        computeErrorAndUpdateHeap(fit);
    }
}


FT TinCreationGreedyInsertionStrategy::errorHeight(const Point_3 &p, const Triangle_3 &t) const {
    // Construct a line in the Z direction
    Line_3 l(p, Vector_3(0, 0, 1));

    // Compute their intersection
    CGAL::cpp11::result_of<Intersect_3(Line_3, Triangle_3)>::type intersect = CGAL::intersection(l, t);

    // If everything goes as expected, the intersection should exist and should be a point
    if (!intersect) {
        std::cerr << "Error! Empty intersection" << std::endl;
        return FT(0.0);
    }
    if (const Segment_3 *s = boost::get<Segment_3>(&*intersect)) {
        std::cerr << "Error! Segment intersection" << std::endl;
        return FT(0.0);
    }

    // Get the intersection point
    const Point_3 *ip = boost::get<Point_3>(&*intersect);

    // Finally, compute the squared distance between the query point and the intersection
    return CGAL::squared_distance(p, *ip);
}


FT TinCreationGreedyInsertionStrategy::error3D(const Point_3 &p, const Triangle_3 &t) const {
    return CGAL::squared_distance(p, t.supporting_plane());
}


void
TinCreationGreedyInsertionStrategy::
insert(const Point_3 &p) {
//    std::cout << "Inserting point = " << p << std::endl;

    // Get the conflict zone in the Delaunay triangulation for the point to be inserted
    std::vector<FaceHandle> facesInConflict;
    m_dt.get_conflicts(p, std::back_inserter(facesInConflict));

    if (facesInConflict.empty()) {
//        // If no faces are in conflict, the point we are trying to add is already in the triangulation! (should not happen...)
        std::cout << "Point already in surface!!!!!!!!!!!!!" << std::endl;
        std::cout << "p = " << p << std::endl;
//        m_heap.pop(); // Do not consider this point anymore
//        return;
    }

//    std::cout << "p = " << p << std::endl;
//    std::cout << "facesInConflict = " << facesInConflict.size() << std::endl;

    // All the faces in the conflict zone will be changed
    // Thus, collect all the points falling in these faces and delete their corresponding entries in the heap
    std::vector<std::shared_ptr<Point_3>> ptsInConflictZone;
    std::vector<FaceHandle>::iterator fit;
    for (fit = facesInConflict.begin(); fit != facesInConflict.end(); fit++) {
        // Collect points' pointers in this face
        std::vector<std::shared_ptr<Point_3>> ptsPtrs = (*(*fit)).info().getPtsSharedPtrs();
        ptsInConflictZone.insert(ptsInConflictZone.end(), ptsPtrs.begin(), ptsPtrs.end());

        // Delete heap entry associated to the face, if any
        if ((*fit)->info().hasHeapNodeHandle()) {
            m_heap.erase((*(*fit)).info().getHeapNodeHandle());
        }

        // Some face handles may not be erased after insertion, clear info as it will be recalculated for the new face shape in case it is not eliminated by insertion
        (*(*fit)).info().clearInfo();
    }

    // Insert the point and modify the triangulation
    VertexHandle vh = m_dt.insert(
            p); // TODO: Use point location once to get a nearby face and then use this face as guess?

    // Check in which faces previously collected points fall
    // While there is no check in this regard, they are all supposed to be falling in the newly created faces that are now
    // covering the previous conflict zone.
    std::vector<std::shared_ptr<Point_3>>::iterator it;
    std::vector<int>::iterator itInd;
    for (it = ptsInConflictZone.begin(); it != ptsInConflictZone.end(); ++it) {
        Point_3 p = *(*it);

        // Locate the triangle containing the current point
        FaceHandle fh = m_dt.locate(p);

        fh->info().addPointPtr(*it);
    }

    // Update the internal structure of the new faces with the points falling on them, and also update the heap
    FaceCirculator fc = m_dt.incident_faces(vh), end(fc);
    do {
        if (!m_dt.is_infinite(fc)) {
            // Compute the maximum error on this face and update the heap
            computeErrorAndUpdateHeap(fc);
        }
    } while (++fc != end);
}


void
TinCreationGreedyInsertionStrategy::
computeErrorAndUpdateHeap(FaceHandle fh) {
    Triangle_3 t = m_dt.triangle(fh);

    // Run over candidates and find the one inducing the worst error
    Point_3 best;
    FT maxSqError = 0;

    std::vector<std::shared_ptr<Point_3>> ptsPtrs = fh->info().getPtsSharedPtrs();
    std::vector<std::shared_ptr<Point_3>>::iterator itPtPtr;
    for (itPtPtr = ptsPtrs.begin();
         itPtPtr != ptsPtrs.end(); ++itPtPtr) {
        Point_3 p = *(*itPtPtr);
        FT e = error(p, t);
        if (e > maxSqError) {
            best = p;
            maxSqError = e;
        }
    }
    if (maxSqError > std::numeric_limits<double>::epsilon() && maxSqError > m_scaledSqApproxTol) {
//        std::cout << "Error = " << maxSqError << std::endl;
//        std::cout << "epsilon = " << std::numeric_limits<double>::epsilon() << std::endl;
        GIHeapNodeHandle nh = m_heap.push(GIHeapNode(maxSqError, best));
//        std::cout << "Best point = " << best << std::endl;
        fh->info().setHeapNodeHandle(nh);
    }
}


FT
TinCreationGreedyInsertionStrategy::
error(const Point_3 &p, const Triangle_3 &t) const {
    if (m_errorType == ErrorHeight)
        return errorHeight(p, t);
    else
        return error3D(p, t);
}


FT
TinCreationGreedyInsertionStrategy::eval(const Point_3& p, const Triangle_3&t) const{
    // Compute the distance to the triangle in the Z direction
    Line_3 l(p, Vector_3(0, 0, 1));

    // Compute their intersection
    CGAL::cpp11::result_of<Intersect_3(Line_3, Triangle_3)>::type intersect = CGAL::intersection(l, t);

    // If everything goes as expected, the intersection should exist and should be a point
    if (!intersect) {
        std::cerr << "Error! Empty intersection" << std::endl;
        return FT(0.0);
    }
    if (const Segment_3 *s = boost::get<Segment_3>(&*intersect)) {
        std::cerr << "Error! Segment intersection" << std::endl;
        return FT(0.0);
    }

    // Get the intersection point
    const Point_3 *ip = boost::get<Point_3>(&*intersect);

    // Finally, compute the squared distance between the query point and the intersection
    FT sqDist = CGAL::squared_distance(p, *ip);
    FT dist = CGAL::sqrt(sqDist);

    // --- Debug (start) ---
//    if (dist > 1000)
//        std::cout << "Distance = " << dist << std::endl;
    // --- Debug (end) ---

    if (p.z() > ip->z())
        dist = -dist;

    return dist;
}


} // End namespace TinCreation