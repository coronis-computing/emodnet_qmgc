//
// Created by ricard on 6/09/17.
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_H
#define EMODNET_TOOLS_QUANTIZED_MESH_H


#include <string>
#include <vector>
#include "cgal_utils.h"
#include <ctb.hpp>


class QuantizedMesh {

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
        unsigned int vertexCount = 0;
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
    QuantizedMesh() {
        m_header = Header() ;
        m_vertexData = VertexData() ;
        m_indexData = IndexData() ;
        m_edgeIndices = EdgeIndices() ;
        m_bytesPerIndex = 0 ;
//        m_hasBounds = false ;
    }


    //! Constructor from file
    QuantizedMesh( const std::string &filePath ) { readFile(filePath) ; }

    //! Read the tile from a file
    bool readFile( const std::string &filePath ) ;

    //! Write the tile to a file
    bool writeFile( const std::string &filePath ) ;

    //! Show the contents of the tile on screen
    void print() ;

    //! Show the contents of the header of the tile on screen
    void printHeader() ;

    /// Set the header part of the quantized mesh structure
    void setHeader( const Header& header ) { m_header = header ; }

    /// Set the vertex data part of the quantized mesh structure
    void setVertexData( const VertexData& vd ) { m_vertexData = vd ; }

    /// Set the index data part of the quantized mesh structure
    void setIndexData( const IndexData& id ) { m_indexData = id ; }

    /// Set the edge indices data part of the quantized mesh structure
    void setEdgeIndices( const EdgeIndices& ei ) { m_edgeIndices = ei ; }

    /// Get the header part of the quantized mesh structure
    Header getHeader() { return m_header ; }

    /// Get the vertex data part of the quantized mesh structure
    VertexData getVertexData() { return m_vertexData ; }

    /// Get the index data of the quantized mesh structure
    IndexData getIndexData() { return m_indexData ; }

    /**
     * \brief Compute the horizon occlusion point for the given tile
     *
     * \p pts Points are supposed to be in ECEF coordinates
     */
    static Point_3 horizonOcclusionPoint( const std::vector<Point_3> &pts, const Point_3 &center ) ;

    // --- Constants ---
    const unsigned short int TILE_SIZE = 65;
    static const unsigned short MAX_VERTEX_DATA = 32767;

    // --- Convenience static functions ---
    static unsigned short remapToVertexDataValue(const double& value, const double& minOr, const double& maxOr) {
//        const unsigned short MAX_VERTEX_DATA = 32767;
        double valueRemap = remap(value, minOr, maxOr, 0, MAX_VERTEX_DATA) ;
//        std::cout << "value = " << value << ", minOr = " << minOr <<", maxOr = " << maxOr << ", valueRemap = " << valueRemap << std::endl ;
        return static_cast<unsigned short>(valueRemap);
    }

    static double remapFromVertexDataValue(const double& value, const double& minOr, const double& maxOr) {
//        const unsigned short MAX_VERTEX_DATA = 32767;
        double valueRemap = remap(value, 0, MAX_VERTEX_DATA, minOr, maxOr ) ;
//        std::cout << "value = " << value << ", minOr = " << minOr <<", maxOr = " << maxOr << ", valueRemap = " << valueRemap << std::endl ;
        return valueRemap;
    }

    static double remap(const double& value, const double& minOr, const double& maxOr, const double& minDest, const double& maxDest) {
        double originalRange = maxOr - minOr ;
        double newRange = maxDest - minDest ;
        double ratio = newRange / originalRange ;
        double newValue = value * ratio ;
        return newValue + minDest ;
    }

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
        return (value >> 1) ^ (-(value & 1)) ;
    }

    /// Encode a value using zig-zag
    unsigned short zigZagEncode( const short &value ) {
        return (value << 1) ^ (value >> 31) ;
    }

//    void createConnectivityAndSimplify(ctb::GDALTile *rasterTile, GDALRasterBand *heightsBand) ;

    // Function as described in https://cesium.com/blog/2013/05/09/computing-the-horizon-occlusion-point/
    // We ommit the ellipsoid here for simplicity (hard-coded)
    static double computeHorizonOcclusionPointMagnitude( const Point_3 &position, const Vector_3 &scaledSpaceDirectionToPoint ) ;


};


#endif //EMODNET_TOOLS_QUANTIZED_MESH_H
