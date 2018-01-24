//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#include "quantized_mesh_tiler.h"
#include <GeographicLib/Geocentric.hpp>
#include <algorithm>
#include "tin_creation/tin_creation_cgal_types.h"
//#include "cgal_border_edges_are_constrained_edge_map.h"
//#include "cgal_corner_vertices_are_constrained_vertex_map.h"
//#include "cgal_further_constrained_placement.h"
//#include "cgal_cost_and_count_stop_predicate.h"
#include <CGAL/centroid.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cmath>
#include "meshoptimizer/meshoptimizer.h"
#include <mutex>
#include "crs_conversions.h"


QuantizedMeshTile QuantizedMeshTiler::createTile( const ctb::TileCoordinate &coord,
                                                   std::vector<Point_3> &tileEastVertices,
                                                   std::vector<Point_3> &tileWestVertices,
                                                   std::vector<Point_3> &tileNorthVertices,
                                                   std::vector<Point_3> &tileSouthVertices )
{
    // Get a terrain tile represented by the tile coordinate
    QuantizedMeshTile qmTile(coord, m_options.RefEllipsoid );
//    QuantizedMeshTile* qmTile = new QuantizedMeshTile(coord, m_options.RefEllipsoid );

    float minHeight, maxHeight ;
    ctb::CRSBounds tileBounds ;
    std::vector<Point_3 > uvhPts = getUVHPointsFromRaster(coord,
                                                          tileEastVertices, tileWestVertices,
                                                          tileNorthVertices, tileSouthVertices,
                                                          minHeight, maxHeight, tileBounds );

    // Simplify the surface
    //simplifySurface(surface, tileEastVertices.size() > 0, tileWestVertices.size() > 0, tileNorthVertices.size() > 0, tileSouthVertices.size() > 0) ;
    Polyhedron surface = m_tinCreator.create(uvhPts, tileEastVertices.size() > 0, tileWestVertices.size() > 0, tileNorthVertices.size() > 0, tileSouthVertices.size() > 0) ;

    // Important call! Sorts halfedges such that the non-border edges precede the border edges
    // Needed for the next steps to work properly
    surface.normalize_border() ;

    // Compute the QuantizedMesh header...
    computeQuantizedMeshHeader( qmTile, surface, minHeight, maxHeight, tileBounds ) ;

    // ...and the QuantizedMesh geometry
    computeQuantizedMeshGeometry( qmTile, surface, minHeight, maxHeight, tileEastVertices, tileWestVertices, tileNorthVertices, tileSouthVertices) ;

    return qmTile;
}



std::vector<TinCreation::Point_3> QuantizedMeshTiler::getUVHPointsFromRaster(const ctb::TileCoordinate &coord,
                                                                const std::vector<Point_3> &tileEastVertices,
                                                                const std::vector<Point_3> &tileWestVertices,
                                                                const std::vector<Point_3> &tileNorthVertices,
                                                                const std::vector<Point_3> &tileSouthVertices,
                                                                float& minHeight, float& maxHeight,
                                                                ctb::CRSBounds& tileBounds ) const
{
    m_mutex.lock() ;
    ctb::GDALTile *rasterTile = createRasterTile(coord); // the raster associated with this tile coordinate
    GDALRasterBand *heightsBand = rasterTile->dataset->GetRasterBand(1);
    double noDataValue = heightsBand->GetNoDataValue();
    double resolution;
    tileBounds = terrainTileBounds(coord, resolution);

    // Copy the raster data into an array
    float rasterHeights[m_options.HeighMapSamplingSteps * m_options.HeighMapSamplingSteps];
    if (heightsBand->RasterIO(GF_Read, 0, 0, 256, 256,
                              (void *) rasterHeights,
                              m_options.HeighMapSamplingSteps, m_options.HeighMapSamplingSteps,
                              GDT_Float32, 0, 0) != CE_None) {
        throw ctb::CTBException("Could not read heights from raster");
    }
    m_mutex.unlock() ;

    // Create a base triangulation (using Delaunay) with all the raster info available
    std::vector< Point_3 > heightMapPoints ;

    // Check the start of the rasters: if there are constrained vertices from neighboring tiles to maintain,
    // the western and/or the southern vertices are not touched, and thus we should parse the raster starting from index 1
    bool constrainEastVertices = tileEastVertices.size() > 0 ;
    bool constrainWestVertices = tileWestVertices.size() > 0 ;
    bool constrainNorthVertices = tileNorthVertices.size() > 0 ;
    bool constrainSouthVertices = tileSouthVertices.size() > 0 ;

    int startX = constrainWestVertices? 1: 0 ;
    int endX = constrainEastVertices? m_options.HeighMapSamplingSteps-1: m_options.HeighMapSamplingSteps ;
    int startY = constrainNorthVertices? 1: 0 ;
    int endY = constrainSouthVertices? m_options.HeighMapSamplingSteps-1: m_options.HeighMapSamplingSteps ;

    // Add the vertices from the raster
    for ( int i = startX; i < endX; i++ ) {
        for (int j = startY; j < endY; j++) {
            int y = m_options.HeighMapSamplingSteps - 1 - j; // y coordinate within the tile.
            // Note that the heights in RasterIO have the origin in the upper-left corner,
            // while the tile has it in the lower-left. Obviously, x = i

            // Height value
            float height = rasterHeights[j * m_options.HeighMapSamplingSteps + i];

            // Clipping
            height = clip( height, m_options.ClippingLowValue, m_options.ClippingHighValue ) ;

            // When no data is available, we assume ground data
            if ( height == noDataValue )
                height = 0 ;

            // If the input DEM contains bathymetry, consider the data as depth instead of altitude (negative value!)
            if ( m_options.IsBathymetry )
                height = -height ;

            // In heightmap format
            heightMapPoints.push_back(Point_3(i, y, height));
        }
    }
    // Also, add the vertices to preserve from neighboring tiles
    if ( constrainEastVertices ) {
        for ( std::vector<Point_3>::const_iterator it = tileEastVertices.begin(); it != tileEastVertices.end(); ++it ) {
            // In heightmap format
            heightMapPoints.push_back(*it);
        }
    }
    if ( constrainWestVertices ) {
        for ( std::vector<Point_3>::const_iterator it = tileWestVertices.begin(); it != tileWestVertices.end(); ++it ) {
            // In heightmap format
            heightMapPoints.push_back(*it);
        }
    }
    if ( constrainNorthVertices ) {
        for ( std::vector<Point_3>::const_iterator it = tileNorthVertices.begin(); it != tileNorthVertices.end(); ++it ) {
            // In heightmap format
            heightMapPoints.push_back(*it);
        }
    }
    if ( constrainSouthVertices ) {
        for ( std::vector<Point_3>::const_iterator it = tileSouthVertices.begin(); it != tileSouthVertices.end(); ++it ) {
            heightMapPoints.push_back(*it);
        }
    }

    // Compute min/max height
    minHeight =  std::numeric_limits<float>::infinity() ;
    maxHeight = -std::numeric_limits<float>::infinity() ;
    for ( std::vector<Point_3>::iterator it = heightMapPoints.begin(); it != heightMapPoints.end(); ++it ) {
        // Update max/min height
        if (it->z() < minHeight)
            minHeight = it->z();
        if (it->z() > maxHeight)
            maxHeight = it->z();
    }

    // Encode the points extracted from the raster in u/v/height values in the range [0..1]
    // We simplify the mesh in u/v/h format because they are normalized values and the surface will be better
    // conditioned for simplification
    std::vector< Point_3 > uvhPts ;
    for ( std::vector<Point_3>::iterator it = heightMapPoints.begin(); it != heightMapPoints.end(); ++it ) {
        double u = QuantizedMesh::remap( it->x(), 0.0, m_options.HeighMapSamplingSteps-1, 0.0, 1.0 ) ;
        double v = QuantizedMesh::remap( it->y(), 0.0, m_options.HeighMapSamplingSteps-1, 0.0, 1.0 ) ;
        double h = QuantizedMesh::remap( it->z(), minHeight, maxHeight, 0.0, 1.0 ) ;

        uvhPts.push_back( Point_3( u, v, h ) ) ;
    }

    delete rasterTile;

    return uvhPts ;
}



//void QuantizedMeshTiler::simplifySurface( Polyhedron& surface,
//                                          const bool& constrainEasternVertices,
//                                          const bool& constrainWesternVertices,
//                                          const bool& constrainNorthernVertices,
//                                          const bool& constrainSouthernVertices ) const
//{
//    // Set up the edge constrainer
//    typedef SMS::FurtherConstrainedPlacement<SimplificationPlacement,
//                                            BorderEdgesAreConstrainedEdgeMap,
//                                            CornerVerticesAreConstrainedVertexMap > SimplificationConstrainedPlacement ;
//    BorderEdgesAreConstrainedEdgeMap wsbeac( surface,
//                                             constrainEasternVertices,
//                                             constrainWesternVertices,
//                                             constrainNorthernVertices,
//                                             constrainSouthernVertices ) ;
//    CornerVerticesAreConstrainedVertexMap cvacvm(surface) ;
//    SimplificationConstrainedPlacement scp( wsbeac, cvacvm ) ;
//    SimplificationCost sc( SimplificationCostParams( m_options.SimpWeightVolume,
//                                                     m_options.SimpWeightBoundary,
//                                                     m_options.SimpWeightShape ) ) ;
//
//    // TODO: Find a way to provide an intuitive stop predicate based on cost...
//    //    SMS::Cost_and_count_stop_predicate<Polyhedron> cacsp(m_options.SimpStopCost,
//    //                                                         m_options.SimpStopEdgesCount) ;
//
//    //    typedef SMS::Constrained_placement<SimplificationPlacement, WesternAndSouthernBorderEdgesAreConstrainedEdgeMap > Placement;
//    //    Placement pl(wsbeac) ;
//
//    int r = SMS::edge_collapse
//            ( surface,
//              SimplificationStopPredicate(m_options.SimpStopEdgesCount),
//    //              cacsp,
//              CGAL::parameters::vertex_index_map( get( CGAL::vertex_external_index,surface ) )
//                      .halfedge_index_map(get(CGAL::halfedge_external_index, surface))
//                      .get_cost(sc)
//    //                      .get_placement(pl)
//    //                      .get_placement(SimplificationPlacement())
//                      .get_placement(scp)
//                      .edge_is_constrained_map(wsbeac)
//    ) ;
//
//    // [DEBUG] Write the simplified polyhedron to file
//    //    std::ofstream os("./" + std::to_string(coord.zoom) + "_" + std::to_string(coord.x) + "_" + std::to_string(coord.y) + "_simp.off") ;
//    //    os << surface;
//    //    os.close();
//
//}


void QuantizedMeshTiler::computeQuantizedMeshHeader( QuantizedMeshTile& qmTile,
                                                     const Polyhedron& surface,
                                                     const float& minHeight, float& maxHeight,
                                                     const ctb::CRSBounds& tileBounds ) const
{
    // Convert to lat/lon format
    std::vector<Point_3> latLonPoints;
    for ( Polyhedron::Point_const_iterator it = surface.points_begin(); it != surface.points_end(); ++it ) {
        // In Latitude, Longitude, Height format
        float lat = tileBounds.getMinY() + ((tileBounds.getMaxY() - tileBounds.getMinY()) * it->y());
        float lon = tileBounds.getMinX() + ((tileBounds.getMaxX() - tileBounds.getMinX()) * it->x());
        latLonPoints.push_back(Point_3(lat, lon, it->z()));
    }

    // Points in ECEF coordinates
    std::vector<Point_3> ecefPoints;
    ecefPoints.reserve(latLonPoints.size());
    double minEcefX = std::numeric_limits<double>::infinity();
    double maxEcefX = -std::numeric_limits<double>::infinity();
    double minEcefY = std::numeric_limits<double>::infinity();
    double maxEcefY = -std::numeric_limits<double>::infinity();
    double minEcefZ = std::numeric_limits<double>::infinity();
    double maxEcefZ = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < latLonPoints.size(); i++) {
        double tmpx, tmpy, tmpz;
        crs_conversions::llh2ecef(latLonPoints[i].x(), latLonPoints[i].y(), latLonPoints[i].z(),
                                  tmpx, tmpy, tmpz) ;

        ecefPoints.push_back(Point_3(tmpx, tmpy, tmpz));

        double tmpx2, tmpy2, tmpz2;
        crs_conversions::llh2ecef(latLonPoints[i].x(), latLonPoints[i].y(), latLonPoints[i].z(),
                                  tmpx2, tmpy2, tmpz2) ;

        if (tmpx < minEcefX)
            minEcefX = tmpx;
        if (tmpx > maxEcefX)
            maxEcefX = tmpx;
        if (tmpy < minEcefY)
            minEcefY = tmpy;
        if (tmpy > maxEcefY)
            maxEcefY = tmpy;
        if (tmpz < minEcefZ)
            minEcefZ = tmpz;
        if (tmpz > maxEcefZ)
            maxEcefZ = tmpz;
    }

////     Get the middle point as the center of the bounding box (lat/lon -> ecef)
//    double midLon = tileBounds.getMinX() + ( ( tileBounds.getMaxX() - tileBounds.getMinX() ) / 2 ) ;
//    double midLat = tileBounds.getMinY() + ( ( tileBounds.getMaxY() - tileBounds.getMinY() ) / 2 ) ;
//    double midH = minHeight + ( ( maxHeight - minHeight ) / 2 ) ;
//// Convert to ECEF and store on the structure
//    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()) ;
//    earth.Forward( midLat, midLon, midH, header.CenterX, header.CenterY, header.CenterZ ) ;

    // --- Set all the header info in the quantized mesh format ---
    QuantizedMesh::Header header;
    header.MinimumHeight = minHeight;
    header.MaximumHeight = maxHeight;

    // Get the middle point as the the center of the bounding box (from ecef directly)
    header.CenterX = minEcefX + ((maxEcefX - minEcefX) / 2);
    header.CenterY = minEcefY + ((maxEcefY - minEcefY) / 2);
    header.CenterZ = minEcefZ + ((maxEcefZ - minEcefZ) / 2);

    // Compute the minimum bounding sphere given the points
    TinCreation::MinSphere ms(ecefPoints.begin(), ecefPoints.end());
    header.BoundingSphereRadius = CGAL::to_double(ms.radius());
    header.BoundingSphereCenterX = CGAL::to_double(*ms.center_cartesian_begin());
    header.BoundingSphereCenterY = CGAL::to_double(*(ms.center_cartesian_begin() + 1));
    header.BoundingSphereCenterZ = CGAL::to_double(*(ms.center_cartesian_begin() + 2));

    // Compute the horizon occlusion point
    // Explanation of horizonOcclusion in: https://cesium.com/blog/2013/04/25/horizon-culling/
    // and: https://groups.google.com/forum/#!topic/cesium-dev/8NTW1Wl0d8s
    // Note: The test point for occlusion is scaled within the WGS84 ellipsoid
    Point_3 hop = qmTile.horizonOcclusionPoint(ecefPoints, Point_3(header.CenterX, header.CenterY, header.CenterZ));
    header.HorizonOcclusionPointX = hop.x();
    header.HorizonOcclusionPointY = hop.y();
    header.HorizonOcclusionPointZ = hop.z();

    qmTile.setHeader(header);
}



void QuantizedMeshTiler::computeQuantizedMeshGeometry(QuantizedMeshTile& qmTile,
                                                      Polyhedron& surface,
                                                      const float& minHeight, const float& maxHeight,
                                                      std::vector<Point_3> &tileEastVertices,
                                                      std::vector<Point_3> &tileWestVertices,
                                                      std::vector<Point_3> &tileNorthVertices,
                                                      std::vector<Point_3> &tileSouthVertices) const
{
    // --> VertexData part
    std::vector<unsigned short> vertices ;
    int numVertices = surface.size_of_vertices() ;
    vertices.reserve(numVertices*3) ;
    for ( Polyhedron::Point_const_iterator it = surface.points_begin(); it != surface.points_end(); ++it ) {
        double x = it->x() ;
        double y = it->y() ;
        double z = it->z() ;

        // Truncate values (after simplification, values might be smaller than 0.0 or larger than 1.0
        x = x < 0.0? 0.0 : x ;
        x = x > 1.0? 1.0 : x ;
        y = y < 0.0? 0.0 : y ;
        y = y > 1.0? 1.0 : y ;
        z = z < 0.0? 0.0 : z ;
        z = z > 1.0? 1.0 : z ;

        unsigned short u = QuantizedMesh::remapToVertexDataValue( x, 0.0, 1.0 ) ;
        unsigned short v = QuantizedMesh::remapToVertexDataValue( y, 0.0, 1.0 ) ;
        unsigned short h = QuantizedMesh::remapToVertexDataValue( z, 0.0, 1.0 ) ;

        vertices.push_back(u) ;
        vertices.push_back(v) ;
        vertices.push_back(h) ;
    }

    // --> IndexData part
    QuantizedMesh::IndexData indexData ;

    indexData.triangleCount = surface.size_of_facets() ;
    indexData.indices.reserve(indexData.triangleCount*3) ;
    for ( Polyhedron::Facet_iterator it = surface.facets_begin(); it != surface.facets_end(); ++it) {
        Polyhedron::Halfedge_around_facet_circulator j = it->facet_begin();
        // Facets in our polyhedral surface should be triangles
        CGAL_assertion( CGAL::circulator_size(j) == 3);
        // Extract integer indices
        do {
            indexData.indices.push_back( static_cast<unsigned int>( std::distance(surface.vertices_begin(), j->vertex()) ) );
        } while ( ++j != it->facet_begin() );
    }

    // Optimize the resulting mesh (in order to be able to codify the indices using the "high watermark" method required
    // by the quantized-mesh format, we need to optimize the vertex indices for the cache and fetch
    meshopt_optimizeVertexCache(&indexData.indices[0], &indexData.indices[0], indexData.indices.size(), numVertices, 32 ) ; // Last number is the virtual cache size
    std::vector<unsigned int> vertexRemap(numVertices, ~0u);
    meshopt_optimizeVertexFetch(&vertices[0], &indexData.indices[0], indexData.indices.size(), &vertices[0], numVertices, sizeof(unsigned short)*3, &vertexRemap[0] );

    // Store optimized vertices and indices
    QuantizedMesh::VertexData vertexData ;
    vertexData.vertexCount = surface.size_of_vertices() ;
    vertexData.u.reserve(vertexData.vertexCount) ;
    vertexData.v.reserve(vertexData.vertexCount) ;
    vertexData.height.reserve(vertexData.vertexCount) ;
    for ( int i = 0; i < vertices.size(); i=i+3 ) {
        vertexData.u.push_back(vertices[i]) ;
        vertexData.v.push_back(vertices[i+1]) ;
        vertexData.height.push_back(vertices[i+2]) ;
    }
    qmTile.setVertexData(vertexData) ;
    qmTile.setIndexData(indexData) ;

    // --> EdgeIndices part (also collect the vertices to maintain for this tile)
    QuantizedMesh::EdgeIndices edgeIndices ;
    edgeIndices.westIndices = std::vector<unsigned int>() ;
    edgeIndices.southIndices = std::vector<unsigned int>() ;
    edgeIndices.eastIndices = std::vector<unsigned int>() ;
    edgeIndices.northIndices = std::vector<unsigned int>() ;
    tileWestVertices.clear() ;
    tileSouthVertices.clear() ;
    Point_2 tileCenter( 0.5, 0.5 ) ;
    Point_2 tileLowerLeftCorner( 0.0, 0.0 ) ;
    Point_2 tileUpperLeftCorner( 0.0, 1.0 ) ;
    Point_2 tileLowerRightCorner( 1.0, 0.0 ) ;
    Point_2 tileUpperRightCorner( 1.0, 1.0 ) ;

    // According to the CGAL documentation, normalization: "reorganizes the sequential storage of the edges such that the
    // non-border edges precede the border edges, and that for each border edge the latter one of the two halfedges is a
    // border halfedge (the first one is a non-border halfedge in conformance with the polyhedral surface definition)"
    // Thus, we move along halfedges with an increment of 2
    int numCorners = 0 ; // Just to check correctness

    Polyhedron::Halfedge_iterator e = surface.border_halfedges_begin() ;
    ++e ; // We start at the second halfedge!
    while( e->is_border() )
    {
        // Get the vertex index in the surface structure
        unsigned int vertInd = static_cast<unsigned int>( std::distance(surface.vertices_begin(), e->vertex() ) ) ;
        // Re-map the vertex index to the new index after optimizing for vertex fetching
        vertInd = vertexRemap[vertInd] ;

        // Relevant geometric info of the current edge
        Point_3 p0 = e->vertex()->point() ; // This is the point we will take care of now
        Point_3 p1 = e->prev()->vertex()->point() ; // This is the previous vertex, with which p0 forms an edge

        // Differences between the points in the edge
        double diffX = fabs( p1.x() - p0.x() ) ;
        double diffY = fabs( p1.y() - p0.y() ) ;

        // Check if it is a corner point: the next vertex changes from vertical to horizontal or viceversa
        // If it is a corner point, we should add it twice to the corresponding border

        // Next edge on the border (since we are in a border halfedge, the next operator points to the next halfedge around the "hole"
        Point_3 p2 = e->next()->vertex()->point() ;

        double diffXNext = fabs( p2.x() - p0.x() ) ;
        double diffYNext = fabs( p2.y() - p0.y() ) ;
        bool isCorner = ( ( diffX < diffY ) && ( diffXNext > diffYNext ) ) ||
                        ( ( diffX > diffY ) && ( diffXNext < diffYNext ) ) ;

        // The data must be converted back to double before returning it to update the cache
        // This is because the ranges for height depend on min/max height for each tile and we need to add this vertices as part of the vertices of the new tile, in other bounds
        double x = QuantizedMesh::remap( p0.x(), 0.0, 1.0, 0.0, m_options.HeighMapSamplingSteps - 1);
        double y = QuantizedMesh::remap( p0.y(), 0.0, 1.0, 0.0, m_options.HeighMapSamplingSteps - 1);
        double h = QuantizedMesh::remap( p0.z(), 0.0, 1.0, minHeight, maxHeight);
        Point_3 phm( x, y, h ) ; // Point in "heightmap" format

        if ( isCorner ) {
            numCorners++ ;
            if ( p0.x() < 0.5 && p0.y() < 0.5 ) { // Corner (0, 0)
                edgeIndices.westIndices.push_back(vertInd);
                edgeIndices.southIndices.push_back(vertInd);
                tileWestVertices.push_back(phm) ;
                tileSouthVertices.push_back(phm) ;
            }
            else if ( p0.x() < 0.5 && p0.y() > 0.5 ) { // Corner (0, 1)
                edgeIndices.westIndices.push_back(vertInd);
                edgeIndices.northIndices.push_back(vertInd);
                tileWestVertices.push_back( phm ) ;
                tileNorthVertices.push_back( phm ) ;
            }
            else if ( p0.x() > 0.5 && p0.y() > 0.5 ) { // Corner (1, 1)
                edgeIndices.northIndices.push_back(vertInd);
                edgeIndices.eastIndices.push_back(vertInd);
                tileNorthVertices.push_back( phm ) ;
                tileEastVertices.push_back( phm ) ;
            }
            else { // p0.x() > 0.5 && p0.y() < 0.5 ) // Corner (1, 0)
                edgeIndices.eastIndices.push_back(vertInd);
                edgeIndices.southIndices.push_back(vertInd);
                tileEastVertices.push_back( phm ) ;
                tileSouthVertices.push_back( phm ) ;
            }
        }
        else {
            if (diffX < diffY) {
                // Vertical edge, can be a western or eastern edge
                if (p0.x() < 0.5) {
                    // Western border edge/vertex
                    edgeIndices.westIndices.push_back(vertInd);
                    tileWestVertices.push_back(phm) ;
                } else { // p0.x() >= 0.5
                    // Eastern border vertex
                    edgeIndices.eastIndices.push_back(vertInd);
                    tileEastVertices.push_back(phm) ;
                }
            } else { // diffX >= diffY
                // Horizontal edge, can be a northern or southern edge
                if (p0.y() < 0.5) {
                    // Southern border edge/vertex
                    edgeIndices.southIndices.push_back(vertInd);
                    tileSouthVertices.push_back(phm) ;
                } else { // p0.y() >= 0.5
                    // Northern border edge/vertex
                    edgeIndices.northIndices.push_back(vertInd);
                    tileNorthVertices.push_back(phm) ;
                }
            }
        }

        // Advance 2 positions (i.e., skip non-border halfedges)
        std::advance(e,2) ;
    }

    if ( numCorners != 4 )
        std::cout << "[ERROR] Not all 4 corners of the tile were detected!" << std::endl ;

    edgeIndices.westVertexCount = edgeIndices.westIndices.size() ;
    edgeIndices.southVertexCount = edgeIndices.southIndices.size() ;
    edgeIndices.eastVertexCount = edgeIndices.eastIndices.size() ;
    edgeIndices.northVertexCount = edgeIndices.northIndices.size() ;

    qmTile.setEdgeIndices(edgeIndices) ;

    // Extensions
    // Compute normals
    namespace PMP=CGAL::Polygon_mesh_processing;
    QuantizedMesh::VertexNormals vertexNormals ;
    for ( Polyhedron::Vertex_iterator vit = surface.vertices_begin(); vit != surface.vertices_end(); ++vit ) {
        Vector_3 vn = PMP::compute_vertex_normal(vit, surface) ;

        vertexNormals.nx.push_back((float)vn.x()) ;
        vertexNormals.ny.push_back((float)vn.y()) ;
        vertexNormals.nz.push_back((float)vn.z()) ;
    }

    // Write normals to tile
    qmTile.setVertexNormals(vertexNormals) ;

    //    qmTile->printHeader() ;
//    qmTile->print() ;
}

