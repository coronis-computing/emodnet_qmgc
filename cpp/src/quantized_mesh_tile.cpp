//
// Created by Ricard Campos
//

#include "quantized_mesh_tile.h"
#include <zlib.h>
#include <iostream>
#include <cstring>
#include <vector>



bool QuantizedMeshTile::readFile( const std::string &filePath )
{
    // Open the file
    std::cout << "file = " << filePath.c_str() << std::endl ;

    gzFile terrainFile = gzopen(filePath.c_str(), "rb");
    if (terrainFile == NULL) {
        return false ;
    }

    // Uncompress the file into the buffer
    unsigned char headerBuffer[88];
    int inflatedBytes;
    inflatedBytes = gzread(terrainFile, headerBuffer, 88);
    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;
    gzclose(terrainFile);

    // --- Read the buffer ---

    // Header
    int pos = 0 ;
    memcpy( &m_header.CenterX, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.CenterY, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.CenterZ, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.MinimumHeight, &headerBuffer[pos], 4 ) ;
    pos+=4 ;
    memcpy( &m_header.MaximumHeight, &headerBuffer[pos], 4 ) ;
    pos+=4 ;
    memcpy( &m_header.BoundingSphereCenterX, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.BoundingSphereCenterY, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.BoundingSphereCenterZ, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.BoundingSphereRadius, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.HorizonOcclusionPointX, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.HorizonOcclusionPointY, &headerBuffer[pos], 8 ) ;
    pos+=8 ;
    memcpy( &m_header.HorizonOcclusionPointZ, &headerBuffer[pos], 8 ) ;
    pos+=8 ;

    std::cout << "m_header.CenterX = " << m_header.CenterX << std::endl ;
    std::cout << "m_header.CenterY = " << m_header.CenterY << std::endl ;
    std::cout << "m_header.CenterZ = " << m_header.CenterZ << std::endl ;
    std::cout << "m_header.MinimumHeight = " << m_header.MinimumHeight << std::endl ;
    std::cout << "m_header.MaximumHeight = " << m_header.MaximumHeight << std::endl ;
    std::cout << "m_header.BoundingSphereCenterX = " << m_header.BoundingSphereCenterX << std::endl ;
    std::cout << "m_header.BoundingSphereCenterY = " << m_header.BoundingSphereCenterY << std::endl ;
    std::cout << "m_header.BoundingSphereCenterZ = " << m_header.BoundingSphereCenterZ << std::endl ;
    std::cout << "m_header.BoundingSphereRadius = " << m_header.BoundingSphereRadius << std::endl ;
    std::cout << "m_header.HorizonOcclusionPointX = " << m_header.HorizonOcclusionPointX << std::endl ;
    std::cout << "m_header.HorizonOcclusionPointY = " << m_header.HorizonOcclusionPointY << std::endl ;
    std::cout << "m_header.HorizonOcclusionPointZ = " << m_header.HorizonOcclusionPointZ << std::endl ;

    // Vertex data
    unsigned char vertexCountBuffer[4];
    inflatedBytes = gzread(terrainFile, vertexCountBuffer, 4);
    unsigned int vertexCount ;
    memcpy( &vertexCount, &vertexCountBuffer[0], 4 ) ;
    std::cout << "Number of vertices = " << vertexCount << std::endl ;
    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;

    int bytesPerIndex = 2 ;
    if (vertexCount > 64 * 1024) {
        // More than 64k vertices, so indices are 32-bit.
        bytesPerIndex = 4 ;
    }
    int triangleLength = bytesPerIndex * 3;

    unsigned char vertexBuffer[vertexCount*2*3] ;
    inflatedBytes = gzread(terrainFile, vertexBuffer, vertexCount*2*3);

    std::vector<unsigned short> uVec, vVec, hVec ;
    unsigned short u, v, h ;
    uVec.reserve(vertexCount) ;
    pos = 0 ;
    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
        unsigned short tmp ;
        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
        u += zigZagDecode( tmp ) ;
        uVec.push_back(u) ;
        std::cout << "u = " << u << std::endl ;
    }
    pos+=vertexCount ;

    vVec.reserve(vertexCount) ;
    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
        unsigned short tmp ;
        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
        v += zigZagDecode( tmp ) ;
        vVec.push_back(v) ;
        std::cout << "v = " << v << std::endl ;
    }
    pos+=vertexCount ;

    hVec.reserve(vertexCount) ;
    for ( int i = pos; i < pos+(vertexCount*2); i+=2 ) {
        unsigned short tmp ;
        memcpy( &tmp, &vertexBuffer[i], 2 ) ;
        h += zigZagDecode( tmp ) ;
        hVec.push_back(h) ;
        std::cout << "h = " << h << std::endl ;
    }
    pos+=vertexCount ;

    // Skip over any additional padding that was added for 2/4 byte alignment
    unsigned int bytesToSkip = 0 ;
    if (pos % bytesPerIndex != 0) {
        bytesToSkip = (bytesPerIndex - (pos % bytesPerIndex));
    }
    if (bytesToSkip > 0) {
        std::cout << "Skipping padding bytes (" << bytesToSkip << ")" << std::endl ;
        unsigned char dummyBuffer[4];
        inflatedBytes = gzread(terrainFile, dummyBuffer, bytesToSkip);
    }

    // Faces data
    unsigned char triangleCountBuffer[4];
    inflatedBytes = gzread(terrainFile, triangleCountBuffer, 4);
    unsigned int triangleCount ;
    memcpy( &triangleCount, &triangleCountBuffer[0], 4 ) ;
    pos+=4 ;

    int err ;
    if (inflatedBytes == 0)
        std::cout << "EOF" << std::endl ;
    if (inflatedBytes <  0)
        std::cout << "PROBLEM: " << gzerror(terrainFile, &err) << std::endl ;
        //std::cout << "PROBLEM" << std::endl ;

    std::cout << "Number of triangles = " << triangleCount << std::endl ;
    std::cout << "inflatedBytes = " << inflatedBytes << std::endl ;

    unsigned char triIndBuffer[triangleLength*triangleCount] ;
    inflatedBytes = gzread(terrainFile, triIndBuffer, triangleLength*triangleCount);

    std::vector< unsigned int > triInd ; // While they can be ints or shorts, we store them as ints
    triInd.reserve(triangleCount) ;

    unsigned int highest = 0 ;
    pos = 0 ;
    for ( int i = pos; i < pos+(triangleCount*bytesPerIndex*3); i+=bytesPerIndex ) {
        unsigned int code ;
        if (bytesPerIndex == 2 ) {
            unsigned short tmp ;
            memcpy(&tmp, &triIndBuffer[i], bytesPerIndex);
            code = (unsigned int)tmp ;
        }
        else
            memcpy(&code, &triIndBuffer[i], bytesPerIndex);

        triInd.push_back( highest-code ) ;
        if (code == 0) {
            ++highest;
        }

        std::cout << *(triInd.end()-1) << std::endl ;
    }

    // Edge indices (west)
    unsigned char westVertexCountBuffer[4];
    inflatedBytes = gzread(terrainFile, westVertexCountBuffer, 4);
    unsigned int westVertexCount ;
    memcpy( &westVertexCount, &westVertexCountBuffer[0], 4 ) ;

    std::cout << "west ind count = " << westVertexCount << std::endl ;

    std::vector< unsigned int > westInd ;
    westInd.reserve(westVertexCount) ;
    unsigned char westIndBuffer[westVertexCount*bytesPerIndex] ;
    inflatedBytes = gzread(terrainFile, westIndBuffer, westVertexCount*bytesPerIndex);

    for ( int i = 0; i < (westVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
        unsigned int ind ;
        if (bytesPerIndex == 2 ) {
            unsigned short tmp ;
            memcpy(&tmp, &westIndBuffer[i], bytesPerIndex);
            ind = (unsigned int)tmp ;
        }
        else
            memcpy(&ind, &westIndBuffer[i], bytesPerIndex);
        westInd.push_back(ind) ;
        std::cout << "west ind = " << ind << std::endl ;
    }

    // Edge indices (south)
    unsigned char southVertexCountBuffer[4];
    inflatedBytes = gzread(terrainFile, southVertexCountBuffer, 4);
    unsigned int southVertexCount ;
    memcpy( &southVertexCount, &southVertexCountBuffer[0], 4 ) ;

    std::cout << "south ind count = " << southVertexCount << std::endl ;

    std::vector< unsigned int > southInd ;
    southInd.reserve(southVertexCount) ;
    unsigned char southIndBuffer[southVertexCount*bytesPerIndex] ;
    inflatedBytes = gzread(terrainFile, southIndBuffer, southVertexCount*bytesPerIndex);

    for ( int i = 0; i < (southVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
        unsigned int ind ;
        if (bytesPerIndex == 2 ) {
            unsigned short tmp ;
            memcpy(&tmp, &southIndBuffer[i], bytesPerIndex);
            ind = (unsigned int)tmp ;
        }
        else
            memcpy(&ind, &southIndBuffer[i], bytesPerIndex);
        southInd.push_back(ind) ;
        std::cout << "south ind = " << ind << std::endl ;
    }

    // Edge indices (east)
    unsigned char eastVertexCountBuffer[4];
    inflatedBytes = gzread(terrainFile, eastVertexCountBuffer, 4);
    unsigned int eastVertexCount ;
    memcpy( &eastVertexCount, &eastVertexCountBuffer[0], 4 ) ;

    std::cout << "east ind count = " << eastVertexCount << std::endl ;

    std::vector< unsigned int > eastInd ;
    eastInd.reserve(eastVertexCount) ;
    unsigned char eastIndBuffer[eastVertexCount*bytesPerIndex] ;
    inflatedBytes = gzread(terrainFile, eastIndBuffer, eastVertexCount*bytesPerIndex);

    for ( int i = 0; i < (eastVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
        unsigned int ind ;
        if (bytesPerIndex == 2 ) {
            unsigned short tmp ;
            memcpy(&tmp, &eastIndBuffer[i], bytesPerIndex);
            ind = (unsigned int)tmp ;
        }
        else
            memcpy(&ind, &eastIndBuffer[i], bytesPerIndex);
        eastInd.push_back(ind) ;
        std::cout << "east ind = " << ind << std::endl ;
    }

    // Edge indices (north)
    unsigned char northVertexCountBuffer[4];
    inflatedBytes = gzread(terrainFile, northVertexCountBuffer, 4);
    unsigned int northVertexCount ;
    memcpy( &northVertexCount, &northVertexCountBuffer[0], 4 ) ;

    std::cout << "north ind count = " << northVertexCount << std::endl ;

    std::vector< unsigned int > northInd ;
    northInd.reserve(northVertexCount) ;
    unsigned char northIndBuffer[northVertexCount*bytesPerIndex] ;
    inflatedBytes = gzread(terrainFile, northIndBuffer, northVertexCount*bytesPerIndex);

    for ( int i = 0; i < (northVertexCount*bytesPerIndex); i+=bytesPerIndex ) {
        unsigned int ind ;
        if (bytesPerIndex == 2 ) {
            unsigned short tmp ;
            memcpy(&tmp, &northIndBuffer[i], bytesPerIndex);
            ind = (unsigned int)tmp ;
        }
        else
            memcpy(&ind, &northIndBuffer[i], bytesPerIndex);
        northInd.push_back(ind) ;
        std::cout << "north ind = " << ind << std::endl ;
    }

    return true ;
}