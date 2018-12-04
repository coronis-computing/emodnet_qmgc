// Copyright (c) 2018 Coronis Computing S.L. (Spain)
// All rights reserved.
//
// This file is part of EMODnet Quantized Mesh Generator for Cesium.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Author: Ricard Campos (ricardcd@gmail.com)

#include "quantized_mesh.h"

#include "gzip_file_reader.h"
#include "gzip_file_writer.h"
#include <algorithm>
#include <iostream>
#include <fstream>

bool QuantizedMesh::readFile( const std::string &filePath ) {
    // Open the file
    GZipFileReader reader(filePath);
    if (!reader.isFileOpen()) {
        return false;
    }

    // Header
    m_header.CenterX = reader.readDouble();
    m_header.CenterY = reader.readDouble();
    m_header.CenterZ = reader.readDouble();
    m_header.MinimumHeight = reader.readFloat();
    m_header.MaximumHeight = reader.readFloat();
    m_header.BoundingSphereCenterX = reader.readDouble();
    m_header.BoundingSphereCenterY = reader.readDouble();
    m_header.BoundingSphereCenterZ = reader.readDouble();
    m_header.BoundingSphereRadius = reader.readDouble();
    m_header.HorizonOcclusionPointX = reader.readDouble();
    m_header.HorizonOcclusionPointY = reader.readDouble();
    m_header.HorizonOcclusionPointZ = reader.readDouble();

    // Vertex data
    m_vertexData.vertexCount = reader.readUInt();

    m_bytesPerIndex = 2;
    if (m_vertexData.vertexCount > (64 * 1024)) {
        // More than 64k vertices, so indices are 32-bit.
        m_bytesPerIndex = 4;
    }

    unsigned short u = 0, v = 0, h = 0;
    m_vertexData.u.reserve(m_vertexData.vertexCount);
    for (int i = 0; i < m_vertexData.vertexCount; i++) {
        u += zigZagDecode(reader.readUShort());
        m_vertexData.u.push_back(u);
    }
    m_vertexData.v.reserve(m_vertexData.vertexCount);
    for (int i = 0; i < m_vertexData.vertexCount; i++) {
        v += zigZagDecode(reader.readUShort());
        m_vertexData.v.push_back(v);
    }
    m_vertexData.height.reserve(m_vertexData.vertexCount);
    for (int i = 0; i < m_vertexData.vertexCount; i++) {
        h += zigZagDecode(reader.readUShort());
        m_vertexData.height.push_back(h);
    }

    // Skip over any additional padding that was added for 2/4 byte alignment
    unsigned int bytesToSkip = 0;
    if (reader.getPos() % m_bytesPerIndex != 0) {
        bytesToSkip = (m_bytesPerIndex - (reader.getPos() % m_bytesPerIndex));
    }
    if (bytesToSkip > 0) {
        reader.skipBytes(bytesToSkip);
    }

    // Faces data
    m_indexData.triangleCount = reader.readUInt();
    m_indexData.indices.reserve(m_indexData.triangleCount * 3); // 3 indices per triangle

    // High water mark decoding
    unsigned int highest = 0;
    for (int i = 0; i < m_indexData.triangleCount * 3; i++) {
        unsigned int code;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort();
            code = (unsigned int) tmp;
        } else
            code = reader.readUInt();

        m_indexData.indices.push_back(highest - code);
        if (code == 0) {
            ++highest;
        }
    }

    // Edge indices (west)
    m_edgeIndices.westVertexCount = reader.readUInt();
    m_edgeIndices.westIndices.reserve(m_edgeIndices.westVertexCount);
    for (int i = 0; i < m_edgeIndices.westVertexCount; i++) {
        unsigned int ind;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort();
            ind = (unsigned int) tmp;
        } else
            ind = reader.readUInt();
        m_edgeIndices.westIndices.push_back(ind);
    }

    // Edge indices (south)
    m_edgeIndices.southVertexCount = reader.readUInt();
    m_edgeIndices.southIndices.reserve(m_edgeIndices.southVertexCount);
    for (int i = 0; i < m_edgeIndices.southVertexCount; i++) {
        unsigned int ind;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort();
            ind = (unsigned int) tmp;
        } else
            ind = reader.readUInt();
        m_edgeIndices.southIndices.push_back(ind);
    }

    // Edge indices (east)
    m_edgeIndices.eastVertexCount = reader.readUInt();
    m_edgeIndices.eastIndices.reserve(m_edgeIndices.eastVertexCount);
    for (int i = 0; i < m_edgeIndices.eastVertexCount; i++) {
        unsigned int ind;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort();
            ind = (unsigned int) tmp;
        } else
            ind = reader.readUInt();
        m_edgeIndices.eastIndices.push_back(ind);
    }

    // Edge indices (north)
    m_edgeIndices.northVertexCount = reader.readUInt();
    m_edgeIndices.northIndices.reserve(m_edgeIndices.northVertexCount);
    for (int i = 0; i < m_edgeIndices.northVertexCount; i++) {
        unsigned int ind;
        if (m_bytesPerIndex == 2) {
            unsigned short tmp = reader.readUShort();
            ind = (unsigned int) tmp;
        } else
            ind = reader.readUInt();
        m_edgeIndices.northIndices.push_back(ind);
    }

    // Read extensions, if available
    while (!reader.eof()) {
        // Read extension header
        unsigned char extensionId = reader.readUChar();
        unsigned int extensionLength = reader.readUInt();

        if (reader.eof()) { // TODO: Why do we have to perform this check?
            reader.close() ;
            return true ;
        }

        if ( (int)extensionId == OCT_VERTEX_NORMALS ) {
            m_vertexNormals.nx.reserve(m_vertexData.vertexCount) ;
            m_vertexNormals.ny.reserve(m_vertexData.vertexCount) ;
            m_vertexNormals.nz.reserve(m_vertexData.vertexCount) ;

            for ( int i = 0; i < m_vertexData.vertexCount; i++ ){
                // Read the oct-encoded pair
                unsigned char octxy[2] ;
                octxy[0] = reader.readUChar() ;
                octxy[1] = reader.readUChar() ;

                // Decode to get the 3D floating-point
                float xyz[3] ;
                octDecode(octxy, xyz) ;

                // Store the normal vector
                m_vertexNormals.nx.push_back(xyz[0]) ;
                m_vertexNormals.ny.push_back(xyz[1]) ;
                m_vertexNormals.nz.push_back(xyz[2]) ;
            }
        }
        else if ( (int)extensionId == WATER_MASK ) {
            m_waterMask.mask.reserve(extensionLength) ;
            for ( int i = 0; i < extensionLength; i++ ) {
                m_waterMask.mask.push_back( reader.readUChar() ) ;
            }
        }
        else {
            std::cout << "[WARNING] Unknown extension type with ID = " << (int)extensionId << ", aborting reading at this point" << std::endl ;
            reader.close() ;
            return true ;
        }
    }

    reader.close() ;
    return true ;
}



bool QuantizedMesh::writeFile( const std::string &filePath ) {
    // Open the file for writing
    GZipFileWriter writer(filePath);
    if (!writer.isFileOpen()) {
        return false;
    }

    // Number of bytes per index
    m_bytesPerIndex = 2;
    if (m_vertexData.vertexCount > (64 * 1024)) {
        // More than 64k vertices, so indices are 32-bit.
        m_bytesPerIndex = 4;
    }

    // Header
    writer.writeDouble(m_header.CenterX);
    writer.writeDouble(m_header.CenterY);
    writer.writeDouble(m_header.CenterZ);
    writer.writeFloat(m_header.MinimumHeight);
    writer.writeFloat(m_header.MaximumHeight);
    writer.writeDouble(m_header.BoundingSphereCenterX);
    writer.writeDouble(m_header.BoundingSphereCenterY);
    writer.writeDouble(m_header.BoundingSphereCenterZ);
    writer.writeDouble(m_header.BoundingSphereRadius);
    writer.writeDouble(m_header.HorizonOcclusionPointX);
    writer.writeDouble(m_header.HorizonOcclusionPointY);
    writer.writeDouble(m_header.HorizonOcclusionPointZ);

    // Vertex data
    writer.writeUInt(m_vertexData.vertexCount);

    unsigned short u = zigZagEncode(m_vertexData.u[0]);
    writer.writeUShort(u);
    for (int i = 0; i < m_vertexData.vertexCount - 1; i++) {
        short ud = m_vertexData.u[i + 1] - m_vertexData.u[i];
        unsigned short ue = zigZagEncode(ud);
        writer.writeUShort(ue);
    }
    unsigned short v = zigZagEncode(m_vertexData.v[0]);
    writer.writeUShort(v);
    for (int i = 0; i < m_vertexData.vertexCount - 1; i++) {
        short vd = m_vertexData.v[i + 1] - m_vertexData.v[i];
        unsigned short ve = zigZagEncode(vd);
        writer.writeUShort(ve);
    }
    unsigned short h = zigZagEncode(m_vertexData.height[0]);
    writer.writeUShort(h);
    for (int i = 0; i < m_vertexData.vertexCount - 1; i++) {
        short hd = m_vertexData.height[i + 1] - m_vertexData.height[i];
        unsigned short he = zigZagEncode(hd);
        writer.writeUShort(he);
    }

    // Add padding for 2/4 byte alignment
    unsigned int bytesToSkip = 0;
    if (writer.getPos() % m_bytesPerIndex != 0) {
        bytesToSkip = (m_bytesPerIndex - (writer.getPos() % m_bytesPerIndex));
    }
    if (bytesToSkip > 0) {
        for (int i = 0; i < bytesToSkip; i++) {
            unsigned char dummy = 0;
            writer.writeByte(dummy);
        }
    }

    // Faces data
    writer.writeUInt(m_indexData.triangleCount);

    // High water mark encoding
    if (m_bytesPerIndex == 2) {
        unsigned short highest = 0;
        for (int i = 0; i < m_indexData.triangleCount * 3; i++) {
            unsigned short code = highest - m_indexData.indices[i]; // Can this overflow? --> No if using vertex cache optimization...

            writer.writeUShort(code);

            if (code == 0) {
                ++highest;
            }
        }
    } else {
        unsigned int highest = 0;
        for (int i = 0; i < m_indexData.triangleCount * 3; i++) {
            unsigned int code = highest - m_indexData.indices[i]; // CHECK! Can this overflow?

            writer.writeUInt( code );

            if (code == 0) {
                ++highest;
            }
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

    // Extensions
    if ( m_vertexNormals.nx.size() > 0 ) {
        // --- Oct-Encoded Per-Vertex Normals extension ---
        // Write extension header
        writer.writeUChar((int)1) ; // extensionId
        writer.writeUInt(m_vertexData.vertexCount*2*8) ; // extensionLength

        // Write oct-encoded normals
        for ( int i = 0; i < m_vertexNormals.nx.size(); i++ ) {
            float xyz[3] ;
            xyz[0] = m_vertexNormals.nx[i] ;
            xyz[1] = m_vertexNormals.ny[i] ;
            xyz[2] = m_vertexNormals.nz[i] ;

            unsigned char octxy[2] ;
            octEncode(xyz, octxy) ;

            writer.writeUChar(octxy[0]) ;
            writer.writeUChar(octxy[1]) ;
        }
    }
    if ( m_waterMask.mask.size() > 0 ) {
        // --- Water Mask extension ---
        // Write extension header
        writer.writeUChar((int)2) ; // extensionId
        writer.writeUInt(m_waterMask.mask.size()) ; // extensionLength
        // Write the mask's data
        for ( int i = 0; i < m_waterMask.mask.size(); i++ ) {
            writer.writeUChar(m_waterMask.mask[i]) ;
        }
    }

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

    if ( m_vertexNormals.nx.size() > 0 ) {
        cout << "Vertices' Normals:" << endl;
        cout << "  Num. vertices normal = " << m_vertexNormals.nx.size() << endl;
        cout << "  nx =  ";
        for (int i = 0; i < m_vertexNormals.nx.size(); i++)
            cout << m_vertexNormals.nx[i] << " ";
        cout << endl;
        cout << "  ny =  ";
        for (int i = 0; i < m_vertexNormals.ny.size(); i++)
            cout << m_vertexNormals.ny[i] << " ";
        cout << endl;
        cout << "  nz =  ";
        for (int i = 0; i < m_vertexNormals.nz.size(); i++)
            cout << m_vertexNormals.nz[i] << " ";
        cout << endl;
    }

    if ( m_waterMask.mask.size() > 0 ) {
        cout << "Water mask = ";
        for (int i = 0; i < m_waterMask.mask.size(); i++)
            cout << (int)m_waterMask.mask[i] << " ";
        cout << endl;
    }
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
