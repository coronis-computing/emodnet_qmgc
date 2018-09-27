//
// Created by Ricard Campos (rcampos@eia.udg.edu).
//

#include "tin_creation_simplification_point_set.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include "cgal/polyhedron_builder_from_projected_triangulation.h"
//#include "cgal/Polyhedral_mesh_domain_with_features_3_extended.h"
// Project-related
#include "cgal/detect_sharp_edges_without_borders.h"
#include "cgal/extract_polylines_from_sharp_edges.h"
#include "cgal/extract_tile_borders_from_polyhedron.h"
#include "cgal/cgal_utils.h"
#include "crs_conversions.h"



namespace TinCreation {

Polyhedron TinCreationSimplificationPointSet::create( const std::vector<Point_3>& dataPts,
                                                              const bool &constrainEasternVertices,
                                                              const bool &constrainWesternVertices,
                                                              const bool &constrainNorthernVertices,
                                                              const bool &constrainSouthernVertices)
{
    // Scale the parameters according to the tile
    m_borderSimpMaxScaledSqDist = m_borderSimpMaxDist*this->getScaleZ();
    m_borderSimpMaxScaledSqDist *= m_borderSimpMaxScaledSqDist; // Squared value to ease distance computations
//    m_borderSimpMaxLengthPercent; // Do not scale by the Z! this is in XY

    PointCloud ptsToSimplify ;

    // Delaunay triangulation
    Delaunay dt( dataPts.begin(), dataPts.end() );

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilderFromProjectedTriangulation<Delaunay, HalfedgeDS> builderDT(dt);
    surface.delegate(builderDT);
    surface.normalize_border();

//    std::cout << "dataPts.size() = " << dataPts.size() << std::endl;

    // Simplification
    getAllNonBorderVertices(surface, ptsToSimplify); // Note that, because of the required pixel overlap for terrain tiles, this additional line of pixels go over the poles in extreme tiles when in ECEF and when converted back they get lat/lon on the other half of the globe... Since we treat border points differently, we don't have any problem. If you try to simplify ALL the points in the tile, the method will fail because of that reason!

//    std::cout << "ptsToSimplify.size() = " << ptsToSimplify.size() << std::endl;

    // --- Debug (start) ---
    // Test metric conversion and viceversa
//    std::cout << "5 Pts to simplify in UVH:" << std::endl;
//    for( int i = 0; i < 5; i++ ) {
//        std::cout << ptsToSimplify[i] << std::endl;
//    }
//    std::vector<Point_3> ptsToSimpECEF = this->convertUVHToECEF(ptsToSimplify);
//    std::cout << "5 Pts to simplify in ECEF:" << std::endl;
//    for( int i = 0; i < 5; i++ ) {
//        std::cout << ptsToSimpECEF[i] << std::endl;
//    }
//    std::vector<Point_3> ptsToSimpUVH = this->convertECEFToUVH(ptsToSimpECEF);
//    std::cout << "5 Pts to simplify converted back to UVH:" << std::endl;
//    for( int i = 0; i < 5; i++ ) {
//        std::cout << ptsToSimpUVH[i] << std::endl;
//    }
    // --- Debug (end) ---

    ptsToSimplify = simplify(ptsToSimplify);

    // Impose the constraints based on borders and features in the original mesh
    imposeConstraints(surface,
                      constrainEasternVertices,
                      constrainWesternVertices,
                      constrainNorthernVertices,
                      constrainSouthernVertices);

    // Insert the simplified points in the constrained triangulation
    for( PointCloud::iterator it = ptsToSimplify.begin(); it != ptsToSimplify.end(); ++it ) {
        m_cdt.insert(*it);
    }

    // Test: Make it conforming Delaunay before storing
//    std::cout << "Making conforming delaunay" << std::endl;
//    CGAL::make_conforming_Delaunay_2(m_cdt);

//    // Translate to Polyhedron
    Polyhedron surface2;
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

    // ------------------------------------------------------------- (uncomment when the detection of polylines is properly implemented)
    // Detect non-border feature polylines
//    std::vector<Polyhedron*> polyPtrsVector(1, &surface);
//    typedef CGAL::Polyhedral_mesh_domain_with_features_3_extended<K>         MeshDomainExt;
//    MeshDomainExt domain(polyPtrsVector.begin(), polyPtrsVector.end());
////    Polylines pls = detect_features_without_border<Polyhedron>(surface, FT(60.0));
//    Polylines featurePolylines = domain.extract_features_without_borders(60.0, surface);
//
//    // Filter small polylines (in terms of its number of points)
//    featurePolylines.erase(std::remove_if(featurePolylines.begin(), featurePolylines.end(),
//                                          [this](const Polyline& pl){return pl.size() < this->m_minFeaturePolylineSize;}),
//                           featurePolylines.end()) ;
//
//    for ( Polylines::const_iterator it = featurePolylines.begin(); it != featurePolylines.end(); ++it ) {
//        m_cdt.insert_constraint((*it).begin(), (*it).end(), false);
//    }

    // -------------------------------------------------------------

    // Create a property map storing if an edge is sharp or not (since the Polyhedron_3 does not have internal property_maps creation, we use a map container within a boost::associative_property_map)
    typedef typename boost::graph_traits<Polyhedron>::edge_descriptor edge_descriptor;
    typedef typename std::map<edge_descriptor, bool> EdgeIsSharpMap;
    typedef typename boost::associative_property_map<EdgeIsSharpMap> EdgeIsSharpPropertyMap;
    EdgeIsSharpPropertyMap eisMap;

    // Detect sharp edges
    detect_sharp_edges_without_borders<Polyhedron, double, EdgeIsSharpPropertyMap, K>(surface, FT(60.0), eisMap);

    // Trace the polylines from the detected edges
    Polylines featurePolylines;
    extract_polylines_from_sharp_edges(surface, eisMap, featurePolylines);

//    int i = 1 ;
//    for (Polylines::const_iterator it = featurePolylines.begin(); it != featurePolylines.end(); ++it, i++ ) {
//        std::cout << "FeaturePolylines{" << i << "} = [ " << std::endl ;
//        for (Polyline::const_iterator itp = (*it).begin(); itp != (*it).end(); ++itp)
//            std::cout << *itp << std::endl;
//        std::cout << "];" << std::endl ;
//    }

    for ( Polylines::const_iterator it = featurePolylines.begin(); it != featurePolylines.end(); ++it ) {
        m_cdt.insert_constraint((*it).begin(), (*it).end(), false);
    }

//    std::cout << "Number of feature Polylines = " << featurePolylines.size() << std::endl ;

//    polylinesToSimplify.insert( polylinesToSimplify.end(), featurePolylines.begin(), featurePolylines.end()) ;
//
//    std::cout << "Number of Polylines to simplify = " << polylinesToSimplify.size() << std::endl ;
//
//    // Simplify the polylines
//    Polylines polylinesSimp = simplifyPolylines(polylinesToSimplify) ;

    std::size_t numRemoved = PS::simplify(m_cdt, PSSqDist3Cost(m_borderSimpMaxLengthPercent), PSStopCost(m_borderSimpMaxScaledSqDist), true);

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

    std::size_t numRemoved = PS::simplify(ct, PSSqDist3Cost(m_borderSimpMaxLengthPercent), PSStopCost(m_borderSimpMaxScaledSqDist), true);

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



std::vector<Point_3>
TinCreationSimplificationPointSet::
convertUVHToECEF(const std::vector<Point_3>& pts) const
{
    if (this->getMaxX() < 0) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return pts;
    }

    // Points in ECEF coordinates
    std::vector<Point_3> ecefPoints;
    ecefPoints.reserve(pts.size());
    for ( std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it )
    {
        // From UVH to lat/lon/height
        double lat = this->getMinY() + ((this->getMaxY() - this->getMinY()) * it->y());
        double lon = this->getMinX() + ((this->getMaxX() - this->getMinX()) * it->x());
        double h   = this->getMinZ() + ((this->getMaxZ() - this->getMinZ()) * it->z());

        double tmpx, tmpy, tmpz;
        crs_conversions::llh2ecef(lat, lon, h,
                                  tmpx, tmpy, tmpz);

        ecefPoints.emplace_back(Point_3(tmpx, tmpy, tmpz));
    }

    return ecefPoints;
}



std::vector<Point_3>
TinCreationSimplificationPointSet::
convertECEFToUVH(const std::vector<Point_3>& pts) const
{
    if (this->getMaxX() < 0) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return pts;
    }

    // Points in ECEF coordinates
    std::vector<Point_3> uvhPoints;
    uvhPoints.reserve(pts.size());
    for ( std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it )
    {
//        std::cout << "x = " << (*it).x() << " / y = " << (*it).y() << " / z = " << (*it).z() << std::endl;

        // From ECEF to lat/lon/height
        double lat, lon, height;
        crs_conversions::ecef2llh((*it).x(), (*it).y(), (*it).z(),
                                  lat, lon, height);

//        if (lat > this->getMaxY())
//            lat = this->getMaxY();
//        else if (lat < this->getMinY())
//            lat = this->getMinY();
//        if (lon > this->getMaxX())
//            lon = lon + this->getMinX();
//        else if (lon < this->getMaxX())
//            lon = this->getMaxX();

//        std::cout << "this->getMinX() = " << this->getMinX() << " / this->getMaxX() = " << this->getMaxX() << std::endl;
//        std::cout << "this->getMinY() = " << this->getMinY() << " / this->getMaxY() = " << this->getMaxY() << std::endl;
//
//        std::cout << "lat = " << lat << " / lon = " << lon << " / h = " << height << std::endl;

        // Scale to local U/V/H
        double u = (lon - this->getMinX()) / (this->getMaxX() - this->getMinX());
        double v = (lat - this->getMinY()) / (this->getMaxY() - this->getMinY());
        double h = (this->getMaxZ() - this->getMinZ()) < std::numeric_limits<double>::epsilon()? // Avoid division by 0 ((this->getMaxZ() - this->getMinZ()) == 0 in flat tiles!)
                         height - this->getMinZ()
                       : (height - this->getMinZ()) / (this->getMaxZ() - this->getMinZ());

//        std::cout << "u = " << u << " / v = " << v << " / h = " << h << std::endl;


//        if ( u > 1 ) u = 0;
//        if ( u < 0 ) u = 1;
//        if ( v > 1 ) v = 1;
//        if ( v < 0 ) v = 0;

//        std::cout << "u = " << u << " / v = " << v << " / h = " << h << std::endl;

        uvhPoints.emplace_back(Point_3(u, v, h));
    }

    return uvhPoints;
}

} // End namespace TinCreation
