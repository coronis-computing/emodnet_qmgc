//
// Created by Ricard Campos
//

#include "quantized_mesh_tile.h"
#include "gzip_file_reader.h"
#include <iostream>
#include <fstream>



//bool QuantizedMeshTile::readFile( const std::string &filePath )
//{
//    // Open the file
//    std::cout << "file = " << filePath.c_str() << std::endl ;
//
//    gzFile terrainFile = gzopen(filePath.c_str(), "rb");
//    if (terrainFile == NULL) {
//        return false ;
//    }
//
//    // Uncompress the file into the buffer
//    unsigned char headerBuffer[88];
//    int inflatedBytes;
//    inflatedBytes = gzread(terrainFile, headerBuffer, 88);
//    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
//    gzclose(terrainFile);
//
//    // --- Read the buffer ---
//
//    // Header
//    int pos = 0 ;
//    memcpy( &m_header.CenterX, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.CenterY, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.CenterZ, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.MinimumHeight, &headerBuffer[pos], 4 ) ;
//    pos+=4 ;
//    memcpy( &m_header.MaximumHeight, &headerBuffer[pos], 4 ) ;
//    pos+=4 ;
//    memcpy( &m_header.BoundingSphereCenterX, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.BoundingSphereCenterY, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.BoundingSphereCenterZ, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.BoundingSphereRadius, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.HorizonOcclusionPointX, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.HorizonOcclusionPointY, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//    memcpy( &m_header.HorizonOcclusionPointZ, &headerBuffer[pos], 8 ) ;
//    pos+=8 ;
//
//    std::cout << "m_header.CenterX = " << m_header.CenterX << std::endl ;
//    std::cout << "m_header.CenterY = " << m_header.CenterY << std::endl ;
//    std::cout << "m_header.CenterZ = " << m_header.CenterZ << std::endl ;
//    std::cout << "m_header.MinimumHeight = " << m_header.MinimumHeight << std::endl ;
//    std::cout << "m_header.MaximumHeight = " << m_header.MaximumHeight << std::endl ;
//    std::cout << "m_header.BoundingSphereCenterX = " << m_header.BoundingSphereCenterX << std::endl ;
//    std::cout << "m_header.BoundingSphereCenterY = " << m_header.BoundingSphereCenterY << std::endl ;
//    std::cout << "m_header.BoundingSphereCenterZ = " << m_header.BoundingSphereCenterZ << std::endl ;
//    std::cout << "m_header.BoundingSphereRadius = " << m_header.BoundingSphereRadius << std::endl ;
//    std::cout << "m_header.HorizonOcclusionPointX = " << m_header.HorizonOcclusionPointX << std::endl ;
//    std::cout << "m_header.HorizonOcclusionPointY = " << m_header.HorizonOcclusionPointY << std::endl ;
//    std::cout << "m_header.HorizonOcclusionPointZ = " << m_header.HorizonOcclusionPointZ << std::endl ;
//
//    // Vertex data
//    unsigned char vertexCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, vertexCountBuffer, 4);
//    unsigned int vertexCount ;
//    memcpy( &vertexCount, &vertexCountBuffer[0], 4 ) ;
//    std::cout << "Number of vertices = " << vertexCount << std::endl ;
//    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
//
//    int bytesPerIndex = 2 ;
//    if (vertexCount > ( 64 * 1024 ) ) {
//        // More than 64k vertices, so indices are 32-bit.
//        bytesPerIndex = 4 ;
//    }
//    std::cout << "Bytes per index = " << bytesPerIndex << std::endl ;
//
//    int triangleLength = bytesPerIndex * 3;
//
//    unsigned char vertexBuffer[vertexCount*2*3] ;
//    inflatedBytes = gzread(terrainFile, vertexBuffer, vertexCount*2*3);
//
//    std::vector<unsigned short> uVec, vVec, hVec ;
//    unsigned short u, v, h ;
//    uVec.reserve(vertexCount) ;
//    pos = 0 ;
//    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
//        unsigned short tmp ;
//        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
//        u += zigZagDecode( tmp ) ;
//        uVec.push_back(u) ;
//        std::cout << "u = " << u << std::endl ;
//    }
//    pos+=vertexCount*2 ;
//
//    vVec.reserve(vertexCount) ;
//    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
//        unsigned short tmp ;
//        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
//        v += zigZagDecode( tmp ) ;
//        vVec.push_back(v) ;
//        std::cout << "v = " << v << std::endl ;
//    }
//    pos+=vertexCount*2 ;
//
//    hVec.reserve(vertexCount) ;
//    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
//        unsigned short tmp ;
//        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
//        h += zigZagDecode( tmp ) ;
//        hVec.push_back(h) ;
//        std::cout << "h = " << h << std::endl ;
//    }
//    pos+=vertexCount*2 ;
//
//    // Skip over any additional padding that was added for 2/4 byte alignment
//    unsigned int bytesToSkip = 0 ;
//    if (pos % bytesPerIndex != 0) {
//        bytesToSkip = (bytesPerIndex - (pos % bytesPerIndex));
//    }
//    if (bytesToSkip > 0) {
//        std::cout << "Skipping padding bytes (" << bytesToSkip << ")" << std::endl ;
//        unsigned char dummyBuffer[4];
//        inflatedBytes = gzread(terrainFile, dummyBuffer, bytesToSkip);
//        std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
//        if (gzeof(terrainFile) == 1)
//            std::cout << "EOF" << std::endl ;
//        int err = 0 ;
//        if (inflatedBytes == -1) { // Z_ERRNO
//            char buffer[ 256 ];
//            char * errorMessage = strerror_r( errno, buffer, 256 ); // get string message from errno
//            std::cout << "PROBLEM ERRNO: " << errorMessage << std::endl;
//        }
//        else {
//            fprintf(stderr, "gzprintf err: %s\n", gzerror(terrainFile, &err));
//            std::cout << "PROBLEM: " << err << std::endl;
//        }
//    }
//
//    // Faces data
//    unsigned char triangleCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, triangleCountBuffer, 4);
//    unsigned int triangleCount ;
//    memcpy( &triangleCount, &triangleCountBuffer[0], 4 ) ;
//    // pos+=4 ;
//
//    int err ;
//    if (inflatedBytes == 0)
//        std::cout << "EOF" << std::endl ;
//    if (inflatedBytes <  0)
//        std::cout << "PROBLEM: " << gzerror(terrainFile, &err) << std::endl ;
//        //std::cout << "PROBLEM" << std::endl ;
//
//    std::cout << "Number of triangles = " << triangleCount << std::endl ;
//    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
//
//    unsigned char triIndBuffer[triangleLength*triangleCount] ;
//    inflatedBytes = gzread(terrainFile, triIndBuffer, triangleLength*triangleCount);
//
//    std::vector< unsigned int > triInd ; // While they can be ints or shorts, we store them as ints
//    triInd.reserve(triangleCount) ;
//
//    // High water mark decoding
//    unsigned int highest = 0 ;
//    for ( int i = 0; i < (triangleLength*triangleCount); i+=bytesPerIndex ) {
//        unsigned int code ;
//        if (bytesPerIndex == 2 ) {
//            unsigned short tmp ;
//            memcpy(&tmp, &triIndBuffer[i], bytesPerIndex);
//            code = (unsigned int)tmp ;
//        }
//        else
//            memcpy(&code, &triIndBuffer[i], bytesPerIndex);
//
//        triInd.push_back( highest-code ) ;
//        if (code == 0) {
//            ++highest;
//        }
//
//        std::cout << *(triInd.end()-1) << std::endl ;
//    }
//
//    // Edge indices (west)
//    unsigned char westVertexCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, westVertexCountBuffer, 4);
//    unsigned int westVertexCount ;
//    memcpy( &westVertexCount, &westVertexCountBuffer[0], 4 ) ;
//
//    if (gzeof(terrainFile) == 1)
//        std::cout << "EOF" << std::endl ;
//
//    std::cout << "west ind count = " << westVertexCount << std::endl ;
//    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
//
//    std::vector< unsigned int > westInd ;
//    westInd.reserve(westVertexCount) ;
//    unsigned char westIndBuffer[westVertexCount*bytesPerIndex] ;
//    inflatedBytes = gzread(terrainFile, westIndBuffer, westVertexCount*bytesPerIndex);
//
//    for ( int i = 0; i < (westVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
//        unsigned int ind ;
//        if (bytesPerIndex == 2 ) {
//            unsigned short tmp ;
//            memcpy(&tmp, &westIndBuffer[i], bytesPerIndex);
//            ind = (unsigned int)tmp ;
//        }
//        else
//            memcpy(&ind, &westIndBuffer[i], bytesPerIndex);
//        westInd.push_back(ind) ;
//        std::cout << "west ind = " << ind << std::endl ;
//    }
//
//    // Edge indices (south)
//    unsigned char southVertexCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, southVertexCountBuffer, 4);
//    unsigned int southVertexCount ;
//    memcpy( &southVertexCount, &southVertexCountBuffer[0], 4 ) ;
//
//    std::cout << "south ind count = " << southVertexCount << std::endl ;
//
//    std::vector< unsigned int > southInd ;
//    southInd.reserve(southVertexCount) ;
//    unsigned char southIndBuffer[southVertexCount*bytesPerIndex] ;
//    inflatedBytes = gzread(terrainFile, southIndBuffer, southVertexCount*bytesPerIndex);
//
//    for ( int i = 0; i < (southVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
//        unsigned int ind ;
//        if (bytesPerIndex == 2 ) {
//            unsigned short tmp ;
//            memcpy(&tmp, &southIndBuffer[i], bytesPerIndex);
//            ind = (unsigned int)tmp ;
//        }
//        else
//            memcpy(&ind, &southIndBuffer[i], bytesPerIndex);
//        southInd.push_back(ind) ;
//        std::cout << "south ind = " << ind << std::endl ;
//    }
//
//    // Edge indices (east)
//    unsigned char eastVertexCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, eastVertexCountBuffer, 4);
//    unsigned int eastVertexCount ;
//    memcpy( &eastVertexCount, &eastVertexCountBuffer[0], 4 ) ;
//
//    std::cout << "east ind count = " << eastVertexCount << std::endl ;
//
//    std::vector< unsigned int > eastInd ;
//    eastInd.reserve(eastVertexCount) ;
//    unsigned char eastIndBuffer[eastVertexCount*bytesPerIndex] ;
//    inflatedBytes = gzread(terrainFile, eastIndBuffer, eastVertexCount*bytesPerIndex);
//
//    for ( int i = 0; i < (eastVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
//        unsigned int ind ;
//        if (bytesPerIndex == 2 ) {
//            unsigned short tmp ;
//            memcpy(&tmp, &eastIndBuffer[i], bytesPerIndex);
//            ind = (unsigned int)tmp ;
//        }
//        else
//            memcpy(&ind, &eastIndBuffer[i], bytesPerIndex);
//        eastInd.push_back(ind) ;
//        std::cout << "east ind = " << ind << std::endl ;
//    }
//
//    // Edge indices (north)
//    unsigned char northVertexCountBuffer[4];
//    inflatedBytes = gzread(terrainFile, northVertexCountBuffer, 4);
//    unsigned int northVertexCount ;
//    memcpy( &northVertexCount, &northVertexCountBuffer[0], 4 ) ;
//
//    std::cout << "north ind count = " << northVertexCount << std::endl ;
//
//    std::vector< unsigned int > northInd ;
//    northInd.reserve(northVertexCount) ;
//    unsigned char northIndBuffer[northVertexCount*bytesPerIndex] ;
//    inflatedBytes = gzread(terrainFile, northIndBuffer, northVertexCount*bytesPerIndex);
//
//    for ( int i = 0; i < (northVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
//        unsigned int ind ;
//        if (bytesPerIndex == 2 ) {
//            unsigned short tmp ;
//            memcpy(&tmp, &northIndBuffer[i], bytesPerIndex);
//            ind = (unsigned int)tmp ;
//        }
//        else
//            memcpy(&ind, &northIndBuffer[i], bytesPerIndex);
//        northInd.push_back(ind) ;
//        std::cout << "north ind = " << ind << std::endl ;
//    }
//
//    return true ;
//}

bool QuantizedMeshTile::readFile( const std::string &filePath )
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

    unsigned short u, v, h ;
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


//! Show the contents of the tile on screen
void QuantizedMeshTile::print()
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


bool QuantizedMeshTile::exportToOFF( const std::string &outFilePath )
{
    std::ofstream of( outFilePath.c_str(), std::ios_base::out ) ;
    if (!of.is_open())
        return false ;

    // Magic header
    of << "OFF" << std::endl ;

    // Number of vertices and triangles (0 edges, not really used in the format...)
    of << m_vertexData.vertexCount << " " << m_indexData.triangleCount << " 0" << std::endl ;

    // Vertices
    for ( int i = 0; i < m_vertexData.vertexCount; i++ )
    {
        of << m_vertexData.u[i] << " " << m_vertexData.v[i] << " " << m_vertexData.height[i] << std::endl ; // WARNING: Quantized values! Need bounds to get the correct lat-lon values
    }

    // Triangles
    of << "3 " ;
    for ( int i = 0; i < m_indexData.triangleCount*3; i++ )
    {
        of << m_indexData.indices[i] << " ";

        if ( (i+1)%3 == 0 && i < (m_indexData.triangleCount*3)-1 )
            of << std::endl << "3 " ;
    }

    return true ;
}