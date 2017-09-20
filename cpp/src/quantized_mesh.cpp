//
// Created by Ricard Campos
//

#include "quantized_mesh.h"

#include "gzip_file_reader.h"
#include "gzip_file_writer.h"
#include <iostream>
#include <fstream>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/UTMUPS.hpp>



bool QuantizedMesh::readFile( const std::string &filePath )
{
    // Open the file
    GZipFileReader reader( filePath ) ;
    if ( !reader.isFileOpen() ) {
        return false ;
    }

    // Header
    m_header.CenterX = reader.readDouble() ;
    m_header.CenterY = reader.readDouble() ;
    m_header.CenterZ = reader.readDouble() ;
    m_header.MinimumHeight = reader.readFloat() ;
    m_header.MaximumHeight = reader.readFloat() ;
    m_header.BoundingSphereCenterX = reader.readDouble() ;
    m_header.BoundingSphereCenterY = reader.readDouble() ;
    m_header.BoundingSphereCenterZ = reader.readDouble() ;
    m_header.BoundingSphereRadius = reader.readDouble() ;
    m_header.HorizonOcclusionPointX = reader.readDouble() ;
    m_header.HorizonOcclusionPointY = reader.readDouble() ;
    m_header.HorizonOcclusionPointZ = reader.readDouble() ;

    // Vertex data
    m_vertexData.vertexCount = reader.readUInt() ;

    m_bytesPerIndex = 2 ;
    if (m_vertexData.vertexCount  > ( 64 * 1024 ) ) {
        // More than 64k vertices, so indices are 32-bit.
        m_bytesPerIndex = 4 ;
    }

    unsigned short u = 0, v = 0, h = 0 ;
    m_vertexData.u.reserve( m_vertexData.vertexCount ) ;
    for ( int i = 0; i < m_vertexData.vertexCount ; i++ ) {
        u += zigZagDecode( reader.readUShort() ) ;
        m_vertexData.u.push_back(u) ;
    }
    m_vertexData.v.reserve(m_vertexData.vertexCount) ;
    for ( int i = 0; i < m_vertexData.vertexCount; i++ ) {
        v += zigZagDecode( reader.readUShort() ) ;
        m_vertexData.v.push_back(v) ;
    }
    m_vertexData.height.reserve(m_vertexData.vertexCount) ;
    for ( int i = 0; i < m_vertexData.vertexCount; i++ ) {
        h += zigZagDecode( reader.readUShort() ) ;
        m_vertexData.height.push_back(h) ;
    }

    // Skip over any additional padding that was added for 2/4 byte alignment
    unsigned int bytesToSkip = 0 ;
    if (reader.getPos() % m_bytesPerIndex != 0) {
        bytesToSkip = (m_bytesPerIndex - (reader.getPos() % m_bytesPerIndex));
    }
    if (bytesToSkip > 0) {
        reader.skipBytes( bytesToSkip ) ;
    }

    // Faces data
    m_indexData.triangleCount = reader.readUInt() ;
    m_indexData.indices.reserve(m_indexData.triangleCount*3) ; // 3 indices per triangle

    // High water mark decoding
    unsigned int highest = 0 ;
    for ( int i = 0; i < m_indexData.triangleCount*3; i++ ) {
        unsigned int code ;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort() ;
            code = (unsigned int)tmp ;
        }
        else
            code = reader.readUInt() ;

        m_indexData.indices.push_back( highest-code ) ;
        if (code == 0) {
            ++highest ;
        }
    }

    // Edge indices (west)
    m_edgeIndices.westVertexCount = reader.readUInt() ;
    m_edgeIndices.westIndices.reserve(m_edgeIndices.westVertexCount) ;
    for ( int i = 0; i < m_edgeIndices.westVertexCount; i++ ) {
        unsigned int ind ;
        if (m_bytesPerIndex == 2 ) {
            unsigned short tmp = reader.readUShort() ;
            ind = (unsigned int)tmp ;
        }
        else
            ind = reader.readUInt() ;
        m_edgeIndices.westIndices.push_back(ind) ;
    }

    // Edge indices (south)
    m_edgeIndices.southVertexCount = reader.readUInt() ;
    m_edgeIndices.southIndices.reserve(m_edgeIndices.southVertexCount) ;
    for ( int i = 0; i < m_edgeIndices.southVertexCount; i++ ) {
        unsigned int ind ;
        if (m_bytesPerIndex == 2 ) {
            unsigned short tmp = reader.readUShort() ;
            ind = (unsigned int)tmp ;
        }
        else
            ind = reader.readUInt() ;
        m_edgeIndices.southIndices.push_back(ind) ;
    }

    // Edge indices (east)
    m_edgeIndices.eastVertexCount = reader.readUInt() ;
    m_edgeIndices.eastIndices.reserve(m_edgeIndices.eastVertexCount) ;
    for ( int i = 0; i < m_edgeIndices.eastVertexCount; i++ ) {
        unsigned int ind ;
        if (m_bytesPerIndex == 2 ) {
            unsigned short tmp = reader.readUShort() ;
            ind = (unsigned int)tmp ;
        }
        else
            ind = reader.readUInt() ;
        m_edgeIndices.eastIndices.push_back(ind) ;
    }

    // Edge indices (north)
    m_edgeIndices.northVertexCount = reader.readUInt() ;
    m_edgeIndices.northIndices.reserve(m_edgeIndices.northVertexCount) ;
    for ( int i = 0; i < m_edgeIndices.northVertexCount; i++ ) {
        unsigned int ind ;
        if (m_bytesPerIndex == 2 ) {
            unsigned short tmp = reader.readUShort() ;
            ind = (unsigned int)tmp ;
        }
        else
            ind = reader.readUInt() ;
        m_edgeIndices.northIndices.push_back(ind) ;
    }

    // TODO: Read extensions!

    reader.close() ;

    return true ;
}



bool QuantizedMesh::writeFile( const std::string &filePath )
{
    // Open the file for writing
    GZipFileWriter writer( filePath ) ;
    if ( !writer.isFileOpen() ) {
        return false ;
    }

    // Number of bytes per index
    m_bytesPerIndex = 2 ;
    if (m_vertexData.vertexCount  > ( 64 * 1024 ) ) {
        // More than 64k vertices, so indices are 32-bit.
        m_bytesPerIndex = 4 ;
    }

    // Header
    writer.writeDouble( m_header.CenterX ) ;
    writer.writeDouble( m_header.CenterY ) ;
    writer.writeDouble( m_header.CenterZ ) ;
    writer.writeFloat( m_header.MinimumHeight ) ;
    writer.writeFloat( m_header.MaximumHeight ) ;
    writer.writeDouble( m_header.BoundingSphereCenterX ) ;
    writer.writeDouble( m_header.BoundingSphereCenterY ) ;
    writer.writeDouble( m_header.BoundingSphereCenterZ ) ;
    writer.writeDouble( m_header.BoundingSphereRadius ) ;
    writer.writeDouble( m_header.HorizonOcclusionPointX ) ;
    writer.writeDouble( m_header.HorizonOcclusionPointY ) ;
    writer.writeDouble( m_header.HorizonOcclusionPointZ ) ;

    // Vertex data
    writer.writeUInt( m_vertexData.vertexCount ) ;

    unsigned short u = zigZagEncode( m_vertexData.u[0] ) ;
    writer.writeUShort(u) ;
    for ( int i = 0; i < m_vertexData.vertexCount-1; i++ ) {
        short ud = m_vertexData.u[i+1] - m_vertexData.u[i] ;
        unsigned short ue = zigZagEncode(ud) ;
        writer.writeUShort(ue) ;
    }
    unsigned short v = zigZagEncode( m_vertexData.v[0] ) ;
    writer.writeUShort(v) ;
    for ( int i = 0; i < m_vertexData.vertexCount-1; i++ ) {
        short vd = m_vertexData.v[i+1] - m_vertexData.v[i] ;
        unsigned short ve = zigZagEncode(vd) ;
        writer.writeUShort(ve) ;
    }
    unsigned short h = zigZagEncode( m_vertexData.height[0] ) ;
    writer.writeUShort(h) ;
    for ( int i = 0; i < m_vertexData.vertexCount-1; i++ ) {
        short hd = m_vertexData.height[i+1] - m_vertexData.height[i] ;
        unsigned short he = zigZagEncode(hd) ;
        writer.writeUShort(he) ;
    }

    // Add padding for 2/4 byte alignment
    unsigned int bytesToSkip = 0 ;
    if (writer.getPos() % m_bytesPerIndex != 0) {
        bytesToSkip = (m_bytesPerIndex - (writer.getPos() % m_bytesPerIndex));
    }
    if (bytesToSkip > 0) {
        for ( int i = 0; i < bytesToSkip; i++ ) {
            unsigned char dummy = 0 ;
            writer.writeByte( dummy ) ;
        }
    }

    // Faces data
    writer.writeUInt( m_indexData.triangleCount ) ;

    // High water mark encoding
    unsigned int highest = 0 ;
    for ( int i = 0; i < m_indexData.triangleCount*3; i++ ) {
        unsigned int code = highest - m_indexData.indices[i]; // CHECK! Can this overflow?

        if (m_bytesPerIndex == 2)
            writer.writeUShort((unsigned short)code) ;
        else
            writer.writeUInt(code) ;

        if (code == 0) {
            ++highest ;
        }
    }

    // Edge indices (west)
    writer.writeUInt( m_edgeIndices.westVertexCount ) ;
    for ( int i = 0; i < m_edgeIndices.westVertexCount; i++ ) {
        if (m_bytesPerIndex == 2 )
            writer.writeUShort( (unsigned short)m_edgeIndices.westIndices[i] ) ;
        else
            writer.writeUInt( m_edgeIndices.westIndices[i] ) ;
    }

    // Edge indices (south)
    writer.writeUInt( m_edgeIndices.southVertexCount ) ;
    for ( int i = 0; i < m_edgeIndices.southVertexCount; i++ ) {
        if ( m_bytesPerIndex == 2 )
            writer.writeUShort( (unsigned short)m_edgeIndices.southIndices[i] ) ;
        else
            writer.writeUInt( m_edgeIndices.southIndices[i] ) ;
    }

    // Edge indices (east)
    writer.writeUInt( m_edgeIndices.eastVertexCount ) ;
    for ( int i = 0; i < m_edgeIndices.eastVertexCount; i++ ) {
        if ( m_bytesPerIndex == 2 )
            writer.writeUShort( (unsigned short)m_edgeIndices.eastIndices[i] ) ;
        else
            writer.writeUInt( m_edgeIndices.eastIndices[i] ) ;
    }

    // Edge indices (north)
    writer.writeUInt( m_edgeIndices.northVertexCount ) ;
    for ( int i = 0; i < m_edgeIndices.northVertexCount; i++ ) {
        if ( m_bytesPerIndex == 2 )
            writer.writeUShort( (unsigned short)m_edgeIndices.northIndices[i] ) ;
        else
            writer.writeUInt( m_edgeIndices.northIndices[i] ) ;
    }

    // TODO: Write extensions!

    writer.close() ;

    return true ;

}



//! Show the contents of the tile on screen
void QuantizedMesh::print()
{
    using namespace std ;

    cout << "Header:" << endl ;
    cout << "  CenterX = " << m_header.CenterX << endl ;
    cout << "  CenterY = " << m_header.CenterY << endl ;
    cout << "  CenterZ = " << m_header.CenterZ << endl ;
    cout << "  MinimumHeight = " << m_header.MinimumHeight << endl ;
    cout << "  MaximumHeight = " << m_header.MaximumHeight << endl ;
    cout << "  BoundingSphereCenterX = " << m_header.BoundingSphereCenterX << endl ;
    cout << "  BoundingSphereCenterY = " << m_header.BoundingSphereCenterY << endl ;
    cout << "  BoundingSphereCenterZ = " << m_header.BoundingSphereCenterZ << endl ;
    cout << "  BoundingSphereRadius = " << m_header.BoundingSphereRadius << endl ;
    cout << "  HorizonOcclusionPointX = " << m_header.HorizonOcclusionPointX << endl ;
    cout << "  HorizonOcclusionPointY = " << m_header.HorizonOcclusionPointY << endl ;
    cout << "  HorizonOcclusionPointZ = " << m_header.HorizonOcclusionPointZ << endl ;

    cout << "Vertices:" << std::endl ;
    cout << "  Num. Vertices = " << m_vertexData.vertexCount << std::endl ;
    cout << "  u =  " ;
    for ( int i = 0; i < m_vertexData.vertexCount; i++ )
        cout << m_vertexData.u[i] << " " ;
    cout << endl ;
    cout << "  v =  " ;
    for ( int i = 0; i < m_vertexData.vertexCount; i++ )
        cout << m_vertexData.v[i] << " " ;
    cout << endl ;
    cout << "  height =  " ;
    for ( int i = 0; i < m_vertexData.vertexCount; i++ )
        cout << m_vertexData.height[i] << " " ;
    cout << endl ;

    cout << "Triangles:" << std::endl ;
    cout << "  Num. Triangles = " << m_indexData.triangleCount << std::endl ;
    cout << "  Indices =  " << endl << "    " ;
    for ( int i = 0; i < m_indexData.triangleCount*3; i++ ) {
        cout << m_indexData.indices[i] ;

        if ( (i+1)%3 == 0 && i < m_indexData.triangleCount*3-1 )
            cout << endl << "    " ;
        else
            cout << " " ;
    }
    cout << endl ;

    cout << "Edge indices:" << endl ;
    cout << "  West indices:" << endl ;
    cout << "    Num. indices = " << m_edgeIndices.westVertexCount << endl ;
    cout << "    Indices =  " ;
    for ( int i = 0; i < m_edgeIndices.westVertexCount; i++ )
        cout << m_edgeIndices.westIndices[i] << " " ;
    cout << endl ;
    cout << "  South indices:" << endl ;
    cout << "    Num. indices = " << m_edgeIndices.southVertexCount << endl ;
    cout << "    Indices =  " ;
    for ( int i = 0; i < m_edgeIndices.southVertexCount; i++ )
        cout << m_edgeIndices.southIndices[i] << " " ;
    cout << endl ;
    cout << "  East indices:" << endl ;
    cout << "    Num. indices = " << m_edgeIndices.eastVertexCount << endl ;
    cout << "    Indices =  " ;
    for ( int i = 0; i < m_edgeIndices.eastVertexCount; i++ )
        cout << m_edgeIndices.eastIndices[i] << " " ;
    cout << endl ;
    cout << "  North indices:" << endl ;
    cout << "    Num. indices = " << m_edgeIndices.northVertexCount << endl ;
    cout << "    Indices =  " ;
    for ( int i = 0; i < m_edgeIndices.northVertexCount; i++ )
        cout << m_edgeIndices.northIndices[i] << " " ;
    cout << endl ;

}



void QuantizedMesh::printHeader() {
    using namespace std;

    cout << "Header:" << endl;
    cout << "  CenterX = " << m_header.CenterX << endl;
    cout << "  CenterY = " << m_header.CenterY << endl;
    cout << "  CenterZ = " << m_header.CenterZ << endl;
    cout << "  MinimumHeight = " << m_header.MinimumHeight << endl;
    cout << "  MaximumHeight = " << m_header.MaximumHeight << endl;
    cout << "  BoundingSphereCenterX = " << m_header.BoundingSphereCenterX << endl;
    cout << "  BoundingSphereCenterY = " << m_header.BoundingSphereCenterY << endl;
    cout << "  BoundingSphereCenterZ = " << m_header.BoundingSphereCenterZ << endl;
    cout << "  BoundingSphereRadius = " << m_header.BoundingSphereRadius << endl;
    cout << "  HorizonOcclusionPointX = " << m_header.HorizonOcclusionPointX << endl;
    cout << "  HorizonOcclusionPointY = " << m_header.HorizonOcclusionPointY << endl;
    cout << "  HorizonOcclusionPointZ = " << m_header.HorizonOcclusionPointZ << endl;
}



void QuantizedMesh::createConnectivityAndSimplify()
{
    if ( m_vertexData.vertexCount <= 0 )
        // No data to mesh, return
        return ;


}