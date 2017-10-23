//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#include "quantized_mesh_tiler.h"
#include <GeographicLib/Geocentric.hpp>
#include <algorithm>
#include "cgal_simplification_constrained_borders.h"
#include <cmath>


QuantizedMeshTile *
QuantizedMeshTiler::createTile(const ctb::TileCoordinate &coord,
                               std::vector<Point_3> &tileWestVertices,
                               std::vector<Point_3> &tileSouthVertices ) const {
    // Get a terrain tile represented by the tile coordinate
    QuantizedMeshTile *qmTile = new QuantizedMeshTile(coord);
    ctb::GDALTile *rasterTile = createRasterTile(coord); // the raster associated with this tile coordinate
    GDALRasterBand *heightsBand = rasterTile->dataset->GetRasterBand(1);
    double resolution;
    ctb::CRSBounds tileBounds = terrainTileBounds(coord, resolution);

    // Copy the raster data into an array
    float rasterHeights[heightsBand->GetXSize()*heightsBand->GetYSize()];
    if (heightsBand->RasterIO(GF_Read, 0, 0, heightsBand->GetXSize(), heightsBand->GetYSize(),
                              (void *) rasterHeights,
                              heightsBand->GetXSize(), heightsBand->GetYSize(),
                              GDT_Float32, 0, 0) != CE_None) {
        throw ctb::CTBException("Could not read heights from raster");
    }

    // Create a base triangulation (using Delaunay) with all the raster info available
    std::vector< Point_3 > hMPoints ;
    float minHeight =  999999 ;
    float maxHeight = -999999 ;

    // Check the start of the rasters: if there are constrained vertices from neighboring tiles to maintain,
    // the western and/or the southern vertices are not touched, and thus we should parse the raster starting from index 1
    int startX, startY ;
    if ( tileWestVertices.size() > 0 )
        startX = 1;
    else
        startX = 0 ;
    if ( tileSouthVertices.size() > 0 )
        startY = 1;
    else
        startY = 0 ;

    std::cout << "Collecting raster height samples" << std::endl ;
    std::cout << "startX = " << startX << std::endl ;
    std::cout << "startY = " << startY << std::endl ;

    for ( int i = startX; i < heightsBand->GetXSize(); i++ ) {
        for ( int j = startY; j < heightsBand->GetYSize(); j++ ) {
            float height = rasterHeights[j*heightsBand->GetXSize()+i] ;

//            // WARNING: This is just for EMODNET data, should not be included in the final code...
//            if ( height < -9000 )
//                height = 0 ;
//
//                std::cout << "Height = " << height << std::endl ;

            if ( height < minHeight )
                minHeight = height ;
            else if ( height > maxHeight )
                maxHeight = height ;

            hMPoints.push_back( Point_3(i, (heightsBand->GetYSize()-1)-j, height) ) ;
        }
    }

    // --- Set all the info in the quantized mesh format ---

    // --> Header part

    QuantizedMesh::Header header ;

    // Get the middle point
    double midX = tileBounds.getMinX() + ( ( tileBounds.getMaxX() - tileBounds.getMinX() ) / 2 ) ;
    double midY = tileBounds.getMinY() + ( ( tileBounds.getMaxY() - tileBounds.getMinY() ) / 2 ) ;
    // Convert to ECEF and store on the structure
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()) ;
    earth.Forward( midY, midX, 0, header.CenterX, header.CenterY, header.CenterZ ) ;

    header.MinimumHeight = minHeight ;
    header.MaximumHeight = maxHeight ;

    // Compute the minimum enclosing sphere given the points
    MinSphere ms( hMPoints.begin(), hMPoints.end() ) ;

    header.BoundingSphereRadius = CGAL::to_double( ms.radius() ) ;
    header.BoundingSphereCenterX = CGAL::to_double( *ms.center_cartesian_begin() ) ;
    header.BoundingSphereCenterY = CGAL::to_double( *(ms.center_cartesian_begin()+1) ) ;
    header.BoundingSphereCenterZ = CGAL::to_double( *(ms.center_cartesian_begin()+2) ) ;

    // Explanation of horizonOcclusion in: https://cesium.com/blog/2013/04/25/horizon-culling/
    // and: https://groups.google.com/forum/#!topic/cesium-dev/8NTW1Wl0d8s
    // The test point for occlusion is scaled within the WGS84 ellipsoid

    // Points in ECEF coordinates
    std::vector< Point_3 > ecefPoints ;
    ecefPoints.reserve(hMPoints.size()) ;
    for ( int i = 0; i < hMPoints.size(); i++ ) {
        GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f()) ;
        double tmpx, tmpy, tmpz ;
        earth.Forward( hMPoints[i].y(), hMPoints[i].x(), hMPoints[i].z(), tmpx, tmpy, tmpz ) ;
        ecefPoints.push_back( Point_3( tmpx, tmpy, tmpz )) ;
    }

    Point_3 hop = QuantizedMesh::horizonOcclusionPoint( ecefPoints, Point_3(header.CenterX, header.CenterY, header.CenterZ) ) ;
    header.HorizonOcclusionPointX = hop.x() ;
    header.HorizonOcclusionPointY = hop.y() ;
    header.HorizonOcclusionPointZ = hop.z() ;

//    header.HorizonOcclusionPointX = header.BoundingSphereCenterX / rX ;
//    header.HorizonOcclusionPointY = header.BoundingSphereCenterY / rY ;
//    header.HorizonOcclusionPointZ = header.BoundingSphereCenterZ / rZ ;

    qmTile->setHeader(header) ;

    // --> Create connectivity

    // Encode the points in u/v/height format
    // We simplify the mesh in u/v/h format because they are normalized values and the surface will contain better conditioned triangles
    std::vector< Point_3 > uvhPts ;
    for ( std::vector<Point_3>::iterator it = hMPoints.begin(); it != hMPoints.end(); ++it ) {
        unsigned short u = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->x()), 0, heightsBand->GetXSize()-1) ;
        unsigned short v = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->y()), 0, heightsBand->GetYSize()-1) ;
        unsigned short h = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->z()), minHeight, maxHeight ) ;

        uvhPts.push_back( Point_3( static_cast<double>(u),
                                   static_cast<double>(v),
                                   static_cast<double>(h) ) ) ;
    }

    // Add the neighboring tiles vertices to preserve
    bool constrainWestVertices = false ;
    bool constrainSouthVertices = false ;
    std::cout << "hMPoints.size() = " << hMPoints.size() << std::endl ;
    if ( tileWestVertices.size() > 0 ) {
        // If there are points available
        std::cout << "Inserting western points" << std::endl ;
//        uvhPts.insert( uvhPts.end(), tileWestVertices.begin(), tileWestVertices.end() ) ;
        for ( std::vector<Point_3>::iterator it = tileWestVertices.begin(); it != tileWestVertices.end(); ++it ) {
            Point_3 p( it->x(), it->y(), QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->z()), minHeight, maxHeight ) ) ; // Map the height of the previous tile to the values of this one
            uvhPts.push_back(p);
        }
        constrainWestVertices = true ;
        std::cout << "uvhPts.size() = " << uvhPts.size() << std::endl ;
    }
    if ( tileSouthVertices.size() > 0 ) {
        std::cout << "Inserting southern points" << std::endl;
//        uvhPts.insert( uvhPts.end(), tileSouthVertices.begin(), tileSouthVertices.end());
        for ( std::vector<Point_3>::iterator it = tileSouthVertices.begin(); it != tileSouthVertices.end(); ++it ) {
            Point_3 p( it->x(), it->y(), QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->z()), minHeight, maxHeight ) ) ;
            uvhPts.push_back(p);
        }
        constrainSouthVertices = true ;
        std::cout << "uvhPts.size() = " << uvhPts.size() << std::endl ;
    }

    // Delaunay triangulation
    Delaunay dt( uvhPts.begin(), uvhPts.end() );

    // --- Debug ---
//    delaunayToOFF("./" + std::to_string(coord.zoom) + "_" + std::to_string(coord.x) + "_" + std::to_string(coord.y) + "_dt.off", dt) ;

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilder<Gt, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // --- Simplify the mesh ---
    std::cout << "Simplifying the mesh" << std::endl ;

    // Set up the edge constrainer
    CGALSimplificationConstrainedBorders scb(surface, constrainWestVertices, constrainSouthVertices);

    int r = SMS::edge_collapse
            ( surface, SimplificationStopPredicate(0.1),
              CGAL::parameters::vertex_index_map( get( CGAL::vertex_external_index,surface ) )
                      .halfedge_index_map(get(CGAL::halfedge_external_index, surface))
//                      .get_placement(SMS::Midpoint_placement<Polyhedron>())
                      .get_cost(SimplificationCost())
                      .get_placement(SimplificationPlacement())
                      .edge_is_constrained_map(scb)
            ) ;

    // Write the simplified polyhedron to file
    std::ofstream os("./" + std::to_string(coord.zoom) + "_" + std::to_string(coord.x) + "_" + std::to_string(coord.y) + "_simp.off") ;
    os << surface;
    os.close();

    // --> VertexData part

    QuantizedMesh::VertexData vertexData ;

    vertexData.vertexCount = surface.size_of_vertices() ;
    vertexData.u.reserve(vertexData.vertexCount) ;
    vertexData.v.reserve(vertexData.vertexCount) ;
    vertexData.height.reserve(vertexData.vertexCount) ;

    for ( Polyhedron::Point_iterator it = surface.points_begin(); it != surface.points_end(); ++it ) {
        unsigned short u = static_cast<unsigned short>( it->x() ) ;
        unsigned short v = static_cast<unsigned short>( it->y() ) ;
        unsigned short h = static_cast<unsigned short>( it->z() ) ;

        vertexData.u.push_back(u) ;
        vertexData.v.push_back(v) ;
        vertexData.height.push_back(h) ;
    }

    qmTile->setVertexData(vertexData) ;

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
            indexData.indices.push_back( (int)std::distance(surface.vertices_begin(), j->vertex()) );
        } while ( ++j != it->facet_begin());
    }

    qmTile->setIndexData(indexData) ;

    // --> EdgeIndices part (also collect the vertices to maintain for this tile)
    QuantizedMesh::EdgeIndices edgeIndices ;
    edgeIndices.westIndices = std::vector<unsigned int>() ;
    edgeIndices.southIndices = std::vector<unsigned int>() ;
    edgeIndices.eastIndices = std::vector<unsigned int>() ;
    edgeIndices.northIndices = std::vector<unsigned int>() ;
    tileWestVertices.clear() ;
    tileSouthVertices.clear() ;
    surface.normalize_border() ; // Important call! Sorts halfedges such that the non-border edges precede the border edges
//    const double myEPS = DBL_EPSILON ; // Since the vertices are supposed to be integers, this value can be "large"
    const double myEPS = 0.9 ;
    for ( Polyhedron::Edge_iterator e = surface.border_edges_begin(); e != surface.edges_end(); ++e )
    {
        Point_3 p( e->vertex()->point() ) ;
        int vertInd = (int)std::distance(surface.vertices_begin(), e->vertex() ) ;
        bool alreadyInserted = false ;

        if( p.x() < myEPS ) {
            // Western border vertex
            edgeIndices.westIndices.push_back( vertInd ) ;
        }
        if ( abs( p.x() - QuantizedMesh::MAX_VERTEX_DATA ) < myEPS ) {
            // Eastern border vertex
            edgeIndices.eastIndices.push_back( vertInd ) ;
            // Add this vertex to the list of western vertex to maintain for the next tile
            // Change the X to be 0 (i.e., eastern to western border)
//            std::cout << "( p.x() - QuantizedMesh::MAX_VERTEX_DATA ) = " << ( p.x() - QuantizedMesh::MAX_VERTEX_DATA ) << std::endl ;
//            std::cout << "east vertex = " << p.x() << ", " << p.y() << ", " << p.z() << std::endl ;
            double h = QuantizedMesh::remapFromVertexDataValue( p.z(), minHeight, maxHeight ) ; // The height data must be converted back to double... because the ranges for height depend on min/max height for each tile
            tileWestVertices.push_back( Point_3( 0, p.y(), h ) ) ;
            alreadyInserted = true ;
        }
        if ( p.y() < myEPS ) {
            // Southern border vertex
            edgeIndices.southIndices.push_back( vertInd ) ;
        }
        if ( abs( p.y() - QuantizedMesh::MAX_VERTEX_DATA ) < myEPS ) {
            // Northern border vertex
            edgeIndices.northIndices.push_back( vertInd ) ;
            // Add this vertex to the list of southern vertex to maintain for the tile on the same X on the next row (Y)
            // Change the Y to be 0 (i.e., northern to southern border)
//            std::cout << "north vertex = " << p.x() << ", " << p.y() << ", " << p.z() << std::endl ;
//            if (!alreadyInserted)
            double h = QuantizedMesh::remapFromVertexDataValue( p.z(), minHeight, maxHeight ) ; // The height data must be converted back to double... because the ranges for height depend on min/max height for each tile
                tileSouthVertices.push_back( Point_3( p.x(), 0.0, h ) ) ;
//            else {
//                std::cout << "Hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" << std::endl ;
//                // This is the special case for the corner point in the tile. It is in the south and western border!
//                // it has already been entered in the western border, so we replace its value
//                tileWestVertices[tileWestVertices.size() - 1] = Point_3(0.0, 0.0, p.z());
//            }
        }
    }
    edgeIndices.westVertexCount = edgeIndices.westIndices.size() ;
    edgeIndices.southVertexCount = edgeIndices.southIndices.size() ;
    edgeIndices.eastVertexCount = edgeIndices.eastIndices.size() ;
    edgeIndices.northVertexCount = edgeIndices.northIndices.size() ;

    qmTile->setEdgeIndices(edgeIndices) ;

    qmTile->printHeader() ;
//    qmTile->print() ;

    delete rasterTile;

    return qmTile;
}


ctb::GDALTile *
QuantizedMeshTiler::createRasterTile(const ctb::TileCoordinate &coord) const {
    // Ensure we have some data from which to create a tile
    if (poDataset && poDataset->GetRasterCount() < 1) {
        throw ctb::CTBException("At least one band must be present in the GDAL dataset");
    }

    // Get the bounds and resolution for a tile coordinate which represents the
    // data overlap requested by the terrain specification.
    double resolution;
    ctb::CRSBounds tileBounds = terrainTileBounds(coord, resolution);

    // Convert the tile bounds into a geo transform
    double adfGeoTransform[6];
    adfGeoTransform[0] = tileBounds.getMinX(); // min longitude
    adfGeoTransform[1] = resolution;
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = tileBounds.getMaxY(); // max latitude
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = -resolution;

    ctb::GDALTile *tile = ctb::GDALTiler::createRasterTile(adfGeoTransform);

    // The previous geotransform represented the data with an overlap as required
    // by the terrain specification.  This now needs to be overwritten so that
    // the data is shifted to the bounds defined by tile itself.
    tileBounds = mGrid.tileBounds(coord);
    resolution = mGrid.resolution(coord.zoom);
    adfGeoTransform[0] = tileBounds.getMinX(); // min longitude
    adfGeoTransform[1] = resolution;
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = tileBounds.getMaxY(); // max latitude
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = -resolution;

    // Set the shifted geo transform to the VRT
    if (GDALSetGeoTransform(tile->dataset, adfGeoTransform) != CE_None) {
        throw ctb::CTBException("Could not set geo transform on VRT");
    }

    return tile;
}


QuantizedMeshTiler &
QuantizedMeshTiler::operator=(const QuantizedMeshTiler &other) {
    ctb::GDALTiler::operator=(other);

    return *this;
}


/**
 * \brief Create the tile pyramid in quantized-mesh format
 *
 * Create the tile pyramid in quantized-mesh format. Ensures that the vertices between neighboring tiles in the same
 * zoom are the the same
 *
 */
void QuantizedMeshTiler::createTilePyramid(const int &startZoom, const int &endZoom, const std::string &outDir)
{
    for (ctb::i_zoom zoom = endZoom; zoom <= startZoom; ++zoom) {
        ctb::TileCoordinate ll = this->grid().crsToTile( this->bounds().getLowerLeft(), zoom ) ;
        ctb::TileCoordinate ur = this->grid().crsToTile( this->bounds().getUpperRight(), zoom ) ;

        ctb::TileBounds zoomBounds(ll, ur);

        int numStepsX = (int)zoomBounds.getWidth() + 1 ;

        std::vector<Point_3 > prevTileWesternVertices(0) ; // Stores the vertices to maintain from the previous tile's eastern border. Since we are creating the tiles from left-right, down-up, just the previous one is required
        std::vector< std::vector<Point_3 > > prevRowTilesSouthernVertices( numStepsX, std::vector<Point_3>(0) ) ; // Stores the vertices to maintain of the previous row of tiles' northern borders. Since we process a row each time, we need to store all the vertices of all the tiles from the previous row.
        int tileColumnInd = 0 ;



        // Processing the tiles row-wise, from
        for ( int ty = zoomBounds.getMinY(); ty <= zoomBounds.getMaxY(); ty++ ) {
            for ( int tx = zoomBounds.getMinX(); tx <= zoomBounds.getMaxX(); tx++, tileColumnInd++ ) {
                std::cout << "Processing tile: zoom = " << zoom << ", x = " << tx << ", y = " << ty << std::endl ;

                ctb::TileCoordinate coord( zoom, tx, ty ) ;

                QuantizedMeshTile *terrainTile = this->createTile(coord, prevTileWesternVertices, prevRowTilesSouthernVertices[tileColumnInd] ) ;
                std::cout << "Num western vertices = " << prevTileWesternVertices.size() << std::endl ;
                for (int i = 0; i < prevTileWesternVertices.size(); i++ )
                    std::cout << prevTileWesternVertices[i].x() << ", " << prevTileWesternVertices[i].y() << ", " << prevTileWesternVertices[i].z() << std::endl ;
                std::cout << "Num southern vertices = " << prevRowTilesSouthernVertices[tileColumnInd].size() << std::endl ;
//                for (int i = 0; i < prevRowTilesSouthernVertices[tileColumnInd].size(); i++ )
//                    std::cout << prevRowTilesSouthernVertices[tileColumnInd][i].x() << ", " << prevRowTilesSouthernVertices[tileColumnInd][i].y() << ", " << prevRowTilesSouthernVertices[tileColumnInd][i].z() << std::endl ;

                // write the file
                const std::string fileName = getTileFileAndCreateDirs(coord, outDir);
                terrainTile->writeFile(fileName) ;
            }
            tileColumnInd = 0 ;
            // The first tile on the row does not have to maintain the western edge, clear the list
            prevTileWesternVertices.clear() ;
        }
    }
}