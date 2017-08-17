//
// Created by Ricard Campos
//

#ifndef EMODNET_TOOLS_QUANTIZEDMESHTILE_H
#define EMODNET_TOOLS_QUANTIZEDMESHTILE_H

#include <string>
#include <vector>

class QuantizedMeshTile {

public:

    // Various structs as described in the documentation
    // (https://cesiumjs.org/data-and-assets/terrain/formats/quantized-mesh-1.0.html)
    struct Header
    {
        // The center of the tile in Earth-centered Fixed coordinates.
        double CenterX;
        double CenterY;
        double CenterZ;

        // The minimum and maximum heights in the area covered by this tile.
        // The minimum may be lower and the maximum may be higher than
        // the height of any vertex in this tile in the case that the min/max vertex
        // was removed during mesh simplification, but these are the appropriate
        // values to use for analysis or visualization.
        float MinimumHeight;
        float MaximumHeight;

        // The tile’s bounding sphere.  The X,Y,Z coordinates are again expressed
        // in Earth-centered Fixed coordinates, and the radius is in meters.
        double BoundingSphereCenterX;
        double BoundingSphereCenterY;
        double BoundingSphereCenterZ;
        double BoundingSphereRadius;

        // The horizon occlusion point, expressed in the ellipsoid-scaled Earth-centered Fixed frame.
        // If this point is below the horizon, the entire tile is below the horizon.
        // See http://cesiumjs.org/2013/04/25/Horizon-culling/ for more information.
        double HorizonOcclusionPointX;
        double HorizonOcclusionPointY;
        double HorizonOcclusionPointZ;
    };

    struct VertexData
    {
        unsigned int vertexCount ;
        std::vector<unsigned short> u ;
        std::vector<unsigned short> v ;
        std::vector<unsigned short> height ;
    };

    struct IndexData
    {
        unsigned int triangleCount ;
        std::vector< unsigned int > indices ; // While they can be ints or shorts, we store them as ints
    };

    struct EdgeIndices
    {
        unsigned int westVertexCount ;
        std::vector<unsigned int> westIndices ;

        unsigned int southVertexCount ;
        std::vector<unsigned int> southIndices ;

        unsigned int eastVertexCount;
        std::vector<unsigned int> eastIndices ;

        unsigned int northVertexCount;
        std::vector<unsigned int> northIndices ;
    };

    /* Functions */

    //! Constructor
    QuantizedMeshTile() {
        m_header = Header() ;
        m_vertexData = VertexData() ;
        m_indexData = IndexData() ;
        m_edgeIndices = EdgeIndices() ;
        m_bytesPerIndex = 0 ;
    }

    //! Read the tile from a file
    bool readFile( const std::string &filePath ) ;

    //! Show the contents of the tile on screen
    void print() ;

    //! Export to OFF format
    bool exportToOFF( const std::string &outFilePath ) ;

private:

    // --- Attributes ---
    Header m_header ;
    VertexData m_vertexData ;
    IndexData m_indexData ;
    EdgeIndices m_edgeIndices ;
    int m_bytesPerIndex ; // Number of bytes used per index

    // --- Functions ---

    // Decode a zig-zag encoded value
    unsigned short zigZagDecode( const unsigned short &value ) {
        return (value >> 1) ^ (-(value & 1));
    }

    // Simple linear interpolation function
    double lint( double a, double b, double t )
    {
        return a + (b - a) * t ;
    }

};


#endif //EMODNET_TOOLS_QUANTIZEDMESHTILE_H
