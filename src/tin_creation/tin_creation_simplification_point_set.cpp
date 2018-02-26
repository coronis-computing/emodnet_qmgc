//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
#include "cgal/Polyhedral_mesh_domain_with_features_3_extended.h"
#include "cgal/extract_tile_borders_from_polyhedron.h"
#include "cgal/cgal_utils.h"

namespace TinCreation {

Polyhedron TinCreationSimplificationPointSet::create( const std::vector<Point_3>& dataPts,
                                                              const bool &constrainEasternVertices,
                                                              const bool &constrainWesternVertices,
                                                              const bool &constrainNorthernVertices,
                                                              const bool &constrainSouthernVertices)
{
    PointCloud ptsToSimplify ;

    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
    surface.delegate(builderDT);
    surface.normalize_border();

    // Simplification by clustering using erase-remove idiom
    getAllNonBorderVertices(surface, ptsToSimplify);

    ptsToSimplify = simplify(ptsToSimplify);

    // Impose the constraints based on borders and features in the original mesh
    imposeConstraints(surface,
                      constrainEasternVertices,
                      constrainWesternVertices,
                      constrainNorthernVertices,
                      constrainSouthernVertices);

    // Insert the simplifyed points in the constrained triangulation
    for( PointCloud::iterator it = ptsToSimplify.begin(); it != ptsToSimplify.end(); ++it ) {
        m_cdt.insert(*it);
    }

    // Test: Make it conforming Delaunay before storing
//    std::cout << "Making conforming delaunay" << std::endl;
//    CGAL::make_conforming_Delaunay_2(m_cdt);

//    // Translate to Polyhedron
    Polyhedron surface2 ;
    PolyhedronBuilderFromProjectedTriangulation<CTXY, HalfedgeDS> builderCDT(m_cdt);
    surface2.delegate(builderCDT);

    m_cdt.clear() ; // Clear internal variables for reusing the object
    return surface2 ;
}



void
TinCreationSimplificationPointSet::
imposeConstraints(Polyhedron& surface, // Note: points in the borders to maintain will be removed from this list!
                  const bool &constrainEasternVertices,
                  const bool &constrainWesternVertices,
                  const bool &constrainNorthernVertices,
                  const bool &constrainSouthernVertices)
{
    // NOTE: we do not use the convex hull to get the borders, basically because the implementation of CGAL only
    // considers the corner vertices of the tiles as points on the convex hull, and not the ones ON the edges joining them.
    // We leave the code comented for future reference:
    // Compute the convex hull in the base 2D plane
//    PointCloud chPts;
//    CGAL::convex_hull_2( pts.begin(), pts.end(), std::back_inserter(chPts), CGAL::Projection_traits_xy_3<K>() );

    // Extract the points at the borders
    PointCloud northernBorderVertices, southernBorderVertices, easternBorderVertices, westernBorderVertices ;
    Point_3 cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11;
    extractTileBordersFromPolyhedron<Polyhedron>(surface, easternBorderVertices, westernBorderVertices, northernBorderVertices, southernBorderVertices, cornerPoint00, cornerPoint01, cornerPoint10, cornerPoint11);

    // Sort the points in the borders
    auto smallerThanInX = [](const Point_3& a, const Point_3& b) -> bool {return a.x() < b.x();};
    auto smallerThanInY = [](const Point_3& a, const Point_3& b) -> bool {return a.y() < b.y();};
    std::sort(easternBorderVertices.begin(), easternBorderVertices.end(), smallerThanInY) ;
    std::sort(westernBorderVertices.begin(), westernBorderVertices.end(), smallerThanInY) ;
    std::sort(northernBorderVertices.begin(), northernBorderVertices.end(), smallerThanInX) ;
    std::sort(southernBorderVertices.begin(), southernBorderVertices.end(), smallerThanInX) ;

    // Collect the polylines to simplify, or leave them as they are if required to
    Polylines polylinesToSimplify, polylinesToMaintain ;
    if (!constrainEasternVertices)
//        polylinesToSimplify.push_back(easternBorderVertices);
        m_cdt.insert_constraint(easternBorderVertices.begin(), easternBorderVertices.end(), false);
    else
        polylinesToMaintain.push_back(easternBorderVertices);
    if (!constrainWesternVertices)
//        polylinesToSimplify.push_back(westernBorderVertices );
        m_cdt.insert_constraint(westernBorderVertices.begin(), westernBorderVertices.end(), false);
    else
        polylinesToMaintain.push_back(westernBorderVertices);
    if (!constrainNorthernVertices)
//        polylinesToSimplify.push_back(northernBorderVertices);
        m_cdt.insert_constraint(northernBorderVertices.begin(), northernBorderVertices.end(), false);
    else
        polylinesToMaintain.push_back(northernBorderVertices);
    if (!constrainSouthernVertices)
//        polylinesToSimplify.push_back(southernBorderVertices);
        m_cdt.insert_constraint(southernBorderVertices.begin(), southernBorderVertices.end(), false);
    else
        polylinesToMaintain.push_back(southernBorderVertices);

    // Detect non-border feature polylines
    std::vector<Polyhedron*> polyPtrsVector(1, &surface);
    typedef CGAL::Polyhedral_mesh_domain_with_features_3_extended<K>         MeshDomainExt;
    MeshDomainExt domain(polyPtrsVector.begin(), polyPtrsVector.end());
//    Polylines pls = detect_features_without_border<Polyhedron>(surface, FT(60.0));
    Polylines featurePolylines = domain.extract_features_without_borders(60.0, surface);

    // Filter small polylines (in terms of its number of points)
    featurePolylines.erase(std::remove_if(featurePolylines.begin(), featurePolylines.end(),
                                          [this](const Polyline& pl){return pl.size() < this->m_minFeaturePolylineSize;}),
                           featurePolylines.end()) ;

    for ( Polylines::const_iterator it = featurePolylines.begin(); it != featurePolylines.end(); ++it ) {
        m_cdt.insert_constraint((*it).begin(), (*it).end(), false);
    }

//    int i = 1 ;
//    for (Polylines::const_iterator it = featurePolylines.begin(); it != featurePolylines.end(); ++it, i++ ) {
//        std::cout << "FeaturePolylines{" << i << "} = [ " << std::endl ;
//        for (Polyline::const_iterator itp = (*it).begin(); itp != (*it).end(); ++itp)
//            std::cout << *itp << std::endl;
//        std::cout << "];" << std::endl ;
//    }



//    std::cout << "Number of feature Polylines = " << featurePolylines.size() << std::endl ;

//    polylinesToSimplify.insert( polylinesToSimplify.end(), featurePolylines.begin(), featurePolylines.end()) ;
//
//    std::cout << "Number of Polylines to simplify = " << polylinesToSimplify.size() << std::endl ;
//
//    // Simplify the polylines
//    Polylines polylinesSimp = simplifyPolylines(polylinesToSimplify) ;

    std::size_t numRemoved = PS::simplify(m_cdt, PSSqDist3Cost(), PSStopCost(m_borderSimpMaxSqDist), true);

//    for ( Polylines::iterator it = polylinesSimp.begin(); it != polylinesSimp.end(); ++it ) {
//        ptsToMaintain.insert( ptsToMaintain.end(), it->begin(), it->end() );
//    }

    // Finally, insert the border polylines that need to be maintained as they are
    for ( Polylines::iterator it = polylinesToMaintain.begin(); it != polylinesToMaintain.end(); ++it ) {
        m_cdt.insert_constraint((*it).begin(), (*it).end(), false);
    }
}



//void
//TinCreationSimplificationPointSet::
//extractTileVerticesAtBorders(const Polyhedron& poly,
//                             PointCloud& easternBorderVertices,
//                             PointCloud& westernBorderVertices,
//                             PointCloud& northernBorderVertices,
//                             PointCloud& southernBorderVertices,
//                             Point_3& cornerVertex00,
//                             Point_3& cornerVertex01,
//                             Point_3& cornerVertex10,
//                             Point_3& cornerVertex11) const
//{
//    int numCorners ;
//
//    Polyhedron::Halfedge_const_iterator e = poly.border_halfedges_begin() ;
//    ++e ; // We start at the second halfedge!
//    while( e->is_border() )
//    {
//        // Relevant geometric info of the current edge
//        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
//        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge
//
//        // Differences between the points in the edge
//        double diffX = fabs( p1.x() - p0.x() ) ;
//        double diffY = fabs( p1.y() - p0.y() ) ;
//
//        // Check if it is a corner point: the next vertex changes from vertical to horizontal or viceversa
//        // If it is a corner point, we should add it twice to the corresponding border
//
//        // Next edge on the border (since we are in a border halfedge, the next operator points to the next halfedge around the "hole"
//        Point_3 p2 = e->next()->vertex()->point() ;
//
//        double diffXNext = fabs( p2.x() - p0.x() ) ;
//        double diffYNext = fabs( p2.y() - p0.y() ) ;
//        bool isCorner = ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
//                        ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;
//
//        if ( isCorner ) {
//            numCorners++ ;
//            if ( p0.x() < 0.5 && p0.y() < 0.5 ) { // Corner (0, 0)
//                cornerVertex00 = p0;
//                westernBorderVertices.push_back(p0);
//                southernBorderVertices.push_back(p0);
//            }
//            else if ( p0.x() < 0.5 && p0.y() > 0.5 ) { // Corner (0, 1)
//                cornerVertex01 = p0;
//                westernBorderVertices.push_back(p0);
//                northernBorderVertices.push_back(p0);
//            }
//            else if ( p0.x() > 0.5 && p0.y() > 0.5 ) { // Corner (1, 1)
//                cornerVertex11 = p0;
//                easternBorderVertices.push_back(p0);
//                northernBorderVertices.push_back(p0);
//            }
//            else { // p0.x() > 0.5 && p0.y() < 0.5 ) // Corner (1, 0)
//                cornerVertex10 = p0;
//                easternBorderVertices.push_back(p0);
//                southernBorderVertices.push_back(p0);
//            }
//        }
//        else {
//            if (diffX < diffY) {
//                // Vertical edge, can be a western or eastern edge
//                if (p0.x() < 0.5) {
//                    // Western border edge/vertex
//                    westernBorderVertices.push_back(p0);
//                } else { // p0.x() >= 0.5
//                    // Eastern border vertex
//                    easternBorderVertices.push_back(p0);
//                }
//            } else { // diffX >= diffY
//                // Horizontal edge, can be a northern or southern edge
//                if (p0.y() < 0.5) {
//                    // Southern border edge/vertex
//                    southernBorderVertices.push_back(p0);
//                } else { // p0.y() >= 0.5
//                    // Northern border edge/vertex
//                    northernBorderVertices.push_back(p0);
//                }
//            }
//        }
//
//        // Advance 2 positions (i.e., skip non-border halfedges)
//        std::advance(e,2) ;
//    }
//}



void
TinCreationSimplificationPointSet::
getAllNonBorderVertices(const Polyhedron& poly, PointCloud& nonBorderPts) const
{
    Polyhedron::Vertex_const_iterator vi = poly.vertices_begin() ;
    for (; vi != poly.vertices_end(); ++vi ) {
        if (!isBorder<Polyhedron>(vi))
            nonBorderPts.push_back(vi->point());
    }
}


//PointCloud
//TinCreationSimplificationPointSet::
//simplifyEastOrWestBorder(const PointCloud& pts) const
//{
//    typedef CGAL::Projection_traits_yz_3_extended<K>                         ProjTraitsYZ;
//    typedef PS::Vertex_base_2<ProjTraitsYZ>                                  VbYZ;
//    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsYZ>        FbYZ;
//    typedef CGAL::Triangulation_data_structure_2<VbYZ, FbYZ>                 TDSYZ;
//    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsYZ,
//            TDSYZ, CGAL::Exact_predicates_tag>                               CDTYZ;
//    typedef CGAL::Constrained_triangulation_plus_2<CDTYZ>                    CTYZ;
//
////    // Check the degenerate case where all the points are at 0 elevation
////    int i = 0;
////    for (; i < pts.size(); i++)
////        if ( pts[i].z() > std::numeric_limits<double>::epsilon() )
////            break ;
////    bool degeneratePolyline = ( i != pts.size() );
////
//    CTYZ ctBorder;
//    ctBorder.insert_constraint(pts.begin(), pts.end(), false);
//
//    //PS::simplify(ctBorder, PSCost(), PSStopCost(m_borderSimpMaxSqDist));
//    //std::size_t numRemoved = PS::simplify(ctBorder, PSCost(), PSStopCountRatio(m_borderSimpMaxSqDist), true);
////    if (degeneratePolyline) {
//        std::size_t numRemoved = PS::simplify(ctBorder, PSSqDistCost(), PSStopCost(m_borderSimpMaxSqDist), true);
////    }
////    else {
////        std::size_t numRemoved = PS::simplify(ctBorder, PSScaledSqDistCost(), PSStopCost(m_borderSimpMaxSqDist), true);
////    }
//
//    PointCloud ptsSimp ;
//
////    for(CTXZ::Constraint_iterator cit = ctBorder.constraints_begin();
////        cit != ctBorder.constraints_end();
////        ++cit) {
////        for(CTXZ::Points_in_constraint_iterator vit =
////                ctBorder.points_in_constraint_begin(*cit);
////            vit != ctBorder.points_in_constraint_end(*cit);
////            ++vit)
////            ptsSimp.push_back(*vit);
////    }
//
//    for(CTYZ::Vertices_in_constraint_iterator vit =
//            ctBorder.vertices_in_constraint_begin(*ctBorder.constraints_begin());
//        vit != ctBorder.vertices_in_constraint_end(*ctBorder.constraints_begin());
//        ++vit)
//        ptsSimp.push_back((*vit)->point());
//
//    return ptsSimp;
//}



//PointCloud
//TinCreationSimplificationPointSet::
//simplifyNorthOrSouthBorder(const PointCloud& pts) const
//{
//    typedef CGAL::Projection_traits_xz_3_extended<K>                         ProjTraitsXZ;
//    typedef PS::Vertex_base_2<ProjTraitsXZ>                                  VbXZ;
//    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsXZ>        FbXZ;
//    typedef CGAL::Triangulation_data_structure_2<VbXZ, FbXZ>                 TDSXZ;
//    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsXZ,
//            TDSXZ, CGAL::Exact_predicates_tag>                               CDTXZ;
//    typedef CGAL::Constrained_triangulation_plus_2<CDTXZ>                    CTXZ;
//
////    // Check the degenerate case where all the points are at 0 elevation
////    int i = 0;
////    for (; i < pts.size(); i++)
////        if ( pts[i].z() > std::numeric_limits<double>::epsilon() )
////            break ;
////    bool degeneratePolyline = ( i != pts.size() );
//
//
//    CTXZ ctBorder;
//    ctBorder.insert_constraint(pts.begin(), pts.end(), false);
//
//    //PS::simplify(ctBorder, PSCost(), PSStopCost(m_borderSimpMaxSqDist));
////    std::size_t numRemoved = PS::simplify(ctBorder, PSCost(), PSStopCountRatio(m_borderSimpMaxSqDist), true);
////    if (degeneratePolyline) {
//        std::size_t numRemoved = PS::simplify(ctBorder, PSSqDistCost(), PSStopCost(m_borderSimpMaxSqDist), true);
////    }
////    else {
////        std::size_t numRemoved = PS::simplify(ctBorder, PSScaledSqDistCost(), PSStopCost(m_borderSimpMaxSqDist), true);
////    }
//
//    PointCloud ptsSimp ;
//
////    for(CTXZ::Constraint_iterator cit = ctBorder.constraints_begin();
////        cit != ctBorder.constraints_end();
////        ++cit) {
////        for(CTXZ::Points_in_constraint_iterator vit =
////                ctBorder.points_in_constraint_begin(*cit);
////            vit != ctBorder.points_in_constraint_end(*cit);
////            ++vit)
////            ptsSimp.push_back(*vit);
////    }
//
//    for(CTXZ::Vertices_in_constraint_iterator vit =
//            ctBorder.vertices_in_constraint_begin(*ctBorder.constraints_begin());
//        vit != ctBorder.vertices_in_constraint_end(*ctBorder.constraints_begin());
//        ++vit)
//        ptsSimp.push_back((*vit)->point());
//
//    return ptsSimp;
//}


Polylines
TinCreationSimplificationPointSet::
simplifyPolylines(const Polylines& polylines) const
{
    typedef CGAL::Projection_traits_xy_3_extended<K>                         ProjTraitsXY;
    typedef PS::Vertex_base_2<ProjTraitsXY>                                  VbXY;
    typedef CGAL::Constrained_triangulation_face_base_2<ProjTraitsXY>        FbXY;
    typedef CGAL::Triangulation_data_structure_2<VbXY, FbXY>                 TDSXY;
    typedef CGAL::Constrained_Delaunay_triangulation_2<ProjTraitsXY,
            TDSXY, CGAL::Exact_predicates_tag>                               CDTXY;
    typedef CGAL::Constrained_triangulation_plus_2<CDTXY>                    CTXY;
    typedef PS::Squared_distance_3_cost                                      PSSqDist3Cost;

//    // Check the degenerate case where all the points are at 0 elevation
//    int i = 0;
//    for (; i < pts.size(); i++)
//        if ( pts[i].z() > std::numeric_limits<double>::epsilon() )
//            break ;
//    bool degeneratePolyline = ( i != pts.size() );


    CTXY ct;
    for (Polylines::const_iterator it = polylines.begin(); it != polylines.end(); ++it )
        ct.insert_constraint(it->begin(), it->end(), false);

//    // --- Debug (start) ---
//    int i = 1 ;
//    for (Polylines::const_iterator it = polylines.begin(); it != polylines.end(); ++it, i++ ) {
//        std::cout << "Polylines{" << i << "} = [ " << std::endl ;
//        for (Polyline::const_iterator itp = (*it).begin(); itp != (*it).end(); ++itp)
//            std::cout << *itp << std::endl;
//        std::cout << "];" << std::endl ;
//    }
//    // --- Debug (end) ---

    std::size_t numRemoved = PS::simplify(ct, PSSqDist3Cost(), PSStopCost(m_borderSimpMaxSqDist), true);

    Polylines polylinesSimp ;
    for(CTXY::Constraint_iterator cit = ct.constraints_begin();
        cit != ct.constraints_end();
        ++cit)
    {
        Polyline pl ;
        for(CTXY::Vertices_in_constraint_iterator vit = ct.vertices_in_constraint_begin(*cit);
            vit != ct.vertices_in_constraint_end(*cit);
            ++vit)
            pl.push_back((*vit)->point());
        polylinesSimp.push_back(pl);
    }

    // --- Debug (start) ---
    int i = 1 ;
    for (Polylines::const_iterator it = polylinesSimp.begin(); it != polylinesSimp.end(); ++it, i++ ) {
        std::cout << "PolylinesSimp{" << i << "} = [ " << std::endl ;
        for (Polyline::const_iterator itp = (*it).begin(); itp != (*it).end(); ++itp)
            std::cout << *itp << std::endl;
        std::cout << "];" << std::endl ;
    }
    // --- Debug (end) ---

    return polylinesSimp;
}


} // End namespace TinCreation