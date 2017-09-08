//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#include "quantized_mesh_tiler.h"
#include <GeographicLib/Geocentric.hpp>



QuantizedMeshTile *
QuantizedMeshTiler::createTile(const ctb::TileCoordinate &coord) const {
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
    float minHeight = 999999 ;
    float maxHeight = -999999 ;
    for ( int i = 0; i < heightsBand->GetXSize(); i++ ) {
        for ( int j = 0; j < heightsBand->GetYSize(); j++ ) {
            float height = rasterHeights[i*heightsBand->GetXSize()+j] ;

//            std::cout << "height = " << height << std::endl ;

            if ( height < minHeight )
                minHeight = height ;
            else if ( height > maxHeight )
                maxHeight = height ;

            hMPoints.push_back( Point_3(i, j, height) ) ;
        }
    }

    // Add 4 vertices in the borders
    

    Delaunay dt( hMPoints.begin(), hMPoints.end() );
    std::cout << dt.number_of_vertices() << std::endl;
    std::cout << "here" << std::endl ;

    delaunayToOFF("./BaseTerrainFromRaster.off", dt) ;

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilder<Gt, HalfedgeDS> builder(dt);
    surface.delegate(builder);

//    // --- Simplify the mesh ---
//    // This is a stop predicate (defines when the algorithm terminates).
//    // In this example, the simplification stops when the number of undirected edges
//    // left in the surface mesh drops below the specified number (1000)
//    SMS::Count_stop_predicate<Polyhedron> stop(1000);
//
//    // This the actual call to the simplification algorithm.
//    // The surface mesh and stop conditions are mandatory arguments.
//    // The index maps are needed because the vertices and edges
//    // of this surface mesh lack an "id()" field.
//    int r = SMS::edge_collapse
//            ( surface, stop,
//              CGAL::parameters::vertex_index_map( get( CGAL::vertex_external_index,surface ) )
//                      .halfedge_index_map(get(CGAL::halfedge_external_index, surface))
//                      .get_cost (SMS::Edge_length_cost <Polyhedron>())
//                      .get_placement(SMS::Midpoint_placement<Polyhedron>())
//            ) ;
//
//    // Write the simplified polyhedron to file
//    std::ofstream os("./HeightMapSimplified.off");
//    os << surface;
//    os.close();

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
    MinSphere ms( surface.points_begin(), surface.points_end() ) ;

    header.BoundingSphereRadius = CGAL::to_double( ms.radius() ) ;
    header.BoundingSphereCenterX = CGAL::to_double( *ms.center_cartesian_begin() ) ;
    header.BoundingSphereCenterY = CGAL::to_double( *(ms.center_cartesian_begin()+1) ) ;
    header.BoundingSphereCenterZ = CGAL::to_double( *(ms.center_cartesian_begin()+2) ) ;

    // TODO: Check a good value for this...
    header.HorizonOcclusionPointX = 0.5 ;
    header.HorizonOcclusionPointY = 0.5 ;
    header.HorizonOcclusionPointZ = 0.5 ;

    qmTile->setHeader(header) ;

    // --> VertexData part
    QuantizedMesh::VertexData vertexData ;

    vertexData.vertexCount = surface.size_of_vertices() ;
    vertexData.u.reserve(vertexData.vertexCount) ;
    vertexData.v.reserve(vertexData.vertexCount) ;
    vertexData.height.reserve(vertexData.vertexCount) ;

    // Encode the points in u/v/height format
    const unsigned short maxVertVal = 32767 ;
    for ( Polyhedron::Point_iterator it = surface.points_begin(); it != surface.points_end(); ++it ) {
//        unsigned short u = ( maxVertVal * CGAL::to_double( it->x() ) - tileBounds.getMinX() ) / (tileBounds.getMaxX()-tileBounds.getMinX()) ;
//        unsigned short v = ( maxVertVal * CGAL::to_double( it->y() ) - tileBounds.getMinY() ) / (tileBounds.getMaxY()-tileBounds.getMinY()) ;
//        unsigned short h = ( maxVertVal * CGAL::to_double( it->z() ) - minHeight ) / (maxHeight-minHeight) ;
        unsigned short u = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->x()), 0, heightsBand->GetXSize()) ;
        unsigned short v = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->y()), 0, heightsBand->GetYSize()) ;
        unsigned short h = QuantizedMesh::remapToVertexDataValue( CGAL::to_double(it->z()), minHeight, maxHeight ) ;
        //std::cout << "u = " << u << ", v = " << v << ", h = " << h << std::endl ;
        vertexData.u.push_back(u) ;
        vertexData.v.push_back(v) ;
        vertexData.height.push_back(h) ;
    }

    qmTile->setVertexData(vertexData) ;

    // --> IndexData part
    QuantizedMesh::IndexData indexData ;

    indexData.triangleCount = surface.size_of_facets() ;
    indexData.indices.reserve(indexData.triangleCount*3) ;
    int i = 0 ;
    for ( Polyhedron::Facet_iterator it = surface.facets_begin(); it != surface.facets_end(); ++it) {
        Polyhedron::Halfedge_around_facet_circulator j = it->facet_begin();
        // Facets in our polyhedral surface should be triangles
        CGAL_assertion( CGAL::circulator_size(j) == 3);
        // Extract integer indices
        do {
            indexData.indices[i++] = (int)std::distance(surface.vertices_begin(), j->vertex()) ;
        } while ( ++j != it->facet_begin());
    }

    qmTile->setIndexData(indexData) ;

    // TODO: The edge vertices/indices in the tile... For now we just add dummy values
    QuantizedMesh::EdgeIndices edgeIndices ;

    edgeIndices.westVertexCount = 0 ;
    edgeIndices.westIndices = std::vector<unsigned int>() ;

    edgeIndices.southVertexCount = 0 ;
    edgeIndices.southIndices = std::vector<unsigned int>() ;

    edgeIndices.eastVertexCount = 0 ;
    edgeIndices.eastIndices = std::vector<unsigned int>() ;

    edgeIndices.northVertexCount = 0 ;
    edgeIndices.northIndices = std::vector<unsigned int>() ;

    qmTile->setEdgeIndices(edgeIndices) ;

    qmTile->printHeader() ;

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


ctb::TerrainTiler &
ctb::TerrainTiler::operator=(const ctb::TerrainTiler &other) {
    GDALTiler::operator=(other);

    return *this;
}
