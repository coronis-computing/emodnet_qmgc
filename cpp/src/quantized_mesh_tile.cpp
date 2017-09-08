//
// Created by Ricard Campos
//

#include "quantized_mesh_tile.h"
#include "cgal_defines.h"
#include "cgal_utils.h"
#include <fstream>

QuantizedMeshTile::QuantizedMeshTile(const TileCoordinate &coord):
        QuantizedMesh(),
        Tile(coord)
{}



QuantizedMeshTile::QuantizedMeshTile(const char *fileName, const TileCoordinate &coord):
        QuantizedMesh(fileName),
        Tile(coord)
{}



QuantizedMeshTile::QuantizedMeshTile(const QuantizedMesh &qm, const TileCoordinate &coord):
        QuantizedMesh(qm),
        Tile(coord)
{}


bool QuantizedMeshTile::convertFromHeightMapTile( const std::string &filePath )
{
    // Load the heightmap
    ctb::Terrain hmt(filePath.c_str()) ;

    std::cout << "Terrain readed" << std::endl ;

    // Get the bounds
    const ctb::GlobalGeodetic profile;
    const ctb::CRSBounds tileBounds = profile.tileBounds(*this);

    // Create the points
    std::vector< Point_3 > hMPoints ;

    // Get max/min height values
    float maxHeight, minHeight ;
    maxHeight = -99999 ;
    minHeight = 99999 ;
    for ( int i = 0; i < TILE_SIZE*TILE_SIZE; i++ ) {
        if ( hmt.getHeights()[i] > maxHeight )
            maxHeight = hmt.getHeights()[i] ;
        if ( hmt.getHeights()[i] < minHeight )
            minHeight = hmt.getHeights()[i] ;
    }

    // NED origin
    for ( int i = 0; i < TILE_SIZE; i++ ) {
        for ( int j = 0; j < TILE_SIZE; j++ ) {
            // In heightmap format, heights are encoded as 1/5 meter units above -1000 meters
            // unsigned short height = hmt.getHeights()[i*TILE_SIZE+j] ;
            float height = (float) ( (float)hmt.getHeights()[i*TILE_SIZE+j] / 5. ) - 1000. ;
            // unsigned short height = ( hmt.getHeights()[i*TILE_SIZE+j] + 1000 ) * 5 ;
            float lon = tileBounds.getMinX() + ( tileBounds.getMaxX() - tileBounds.getMinX() ) * ((float)i/TILE_SIZE) ;
            float lat = tileBounds.getMinY() + ( tileBounds.getMaxY() - tileBounds.getMinY() ) * ((float)j/TILE_SIZE) ;

            // Scale height values to be in a similar range of values as lon/lat
            float scaledHeight = ( ( tileBounds.getMaxX() - tileBounds.getMinX() ) * ( height - minHeight ) ) / (maxHeight-minHeight) + tileBounds.getMinX() ;

            hMPoints.push_back( Point_3( lon, lat, scaledHeight ) ) ;

//            GeographicLib::GeoCoords coord( lat, lon ) ;
//            double utmX = coord.Easting() ;
//            double utmY = coord.Northing() ;
//
//            std::cout << "utmX = " << utmX << ", utmY =" << utmY << std::endl ;
//
//            double localX = utmX - enuOrig.Easting() ;
//            double localY = utmY - enuOrig.Northing() ;
//
//            std::cout << "minLat = " << bounds.minLatitude << ", minLon =" << bounds.minLongitude << std::endl ;
//            std::cout << "OrigUtmX = " << enuOrig.Easting() << ", OrigUtmY =" << enuOrig.Northing() << std::endl ;
//
//            hMPoints.push_back( Point_3( localX, localY, height ) ) ;
//            std::cout << "lat = " << lat << ", lon = " << lon << ", localX = " << localX << ", localY = " << localY << ", height = " << height << std::endl ;
        }
    }

    Delaunay dt( hMPoints.begin(), hMPoints.end() );
    std::cout << dt.number_of_vertices() << std::endl;

    delaunayToOFF( "./HeightMapTerrainDelaunay.off", dt ) ;

    // Translate to Polyhedron
    Polyhedron surface ;
    PolyhedronBuilder<Gt, HalfedgeDS> builder(dt);
    surface.delegate(builder);

    // --- Simplify the mesh ---
    // This is a stop predicate (defines when the algorithm terminates).
    // In this example, the simplification stops when the number of undirected edges
    // left in the surface mesh drops below the specified number (1000)
    SMS::Count_stop_predicate<Polyhedron> stop(1000);

    // This the actual call to the simplification algorithm.
    // The surface mesh and stop conditions are mandatory arguments.
    // The index maps are needed because the vertices and edges
    // of this surface mesh lack an "id()" field.
    int r = SMS::edge_collapse
            ( surface, stop,
              CGAL::parameters::vertex_index_map( get( CGAL::vertex_external_index,surface ) )
                      .halfedge_index_map(get(CGAL::halfedge_external_index, surface))
                      .get_cost (SMS::Edge_length_cost <Polyhedron>())
                      .get_placement(SMS::Midpoint_placement<Polyhedron>())
            ) ;

    // Write the simplified polyhedron to file
    std::ofstream os("./HeightMapSimplified.off");
    os << surface;
    os.close();

    return true ;
}



void QuantizedMeshTile::convertUVHToLonLatHeight( const unsigned short &u, const unsigned short &v, const unsigned short &h,
                                                  double &lon, double &lat, double &height )
{
    const ctb::GlobalGeodetic profile;
    const ctb::CRSBounds tileBounds = profile.tileBounds(*this);
    const Header header = getHeader() ;

    lon = tileBounds.getMinX() + ( tileBounds.getMaxX() - tileBounds.getMinX() ) * u/32767 ;
    lat = tileBounds.getMinY() + ( tileBounds.getMaxY() - tileBounds.getMinY() ) * v/32767 ;
    height = header.MinimumHeight + ( header.MaximumHeight - header.MinimumHeight ) * h/32767 ;
}



bool QuantizedMeshTile::exportToOFF( const std::string &outFilePath, const bool& useRealWorldValues )
{
    std::ofstream of( outFilePath.c_str(), std::ios_base::out ) ;
    if (!of.is_open())
        return false ;

    // Magic header
    of << "OFF" << std::endl ;

    IndexData indexData = this->getIndexData() ;
    VertexData vertexData = this->getVertexData() ;

    // Number of vertices and triangles (0 edges, not really used in the format...)
    of << vertexData.vertexCount << " " << indexData.triangleCount << " 0" << std::endl ;

    // Vertices

    for ( int i = 0; i < vertexData.vertexCount; i++ )
    {
        // Convert to Lon/Lat/Height real values from U/V/Height stored in the quantized mesh
        if (useRealWorldValues) {
            double lon, lat, height;
            convertUVHToLonLatHeight(vertexData.u[i], vertexData.v[i], vertexData.height[i], lon, lat, height);
            of << lon << " " << lat << " " << height << std::endl;
        }
        else {
            of << vertexData.u[i] << " " << vertexData.v[i] << " " << vertexData.height[i] << std::endl;
        }
    }

    // Triangles
    of << "3 " ;
    for ( int i = 0; i < indexData.triangleCount*3; i++ )
    {
        of << indexData.indices[i] << " ";

        if ( (i+1)%3 == 0 && i < (indexData.triangleCount*3)-1 )
            of << std::endl << "3 " ;
    }

    return true ;
}
