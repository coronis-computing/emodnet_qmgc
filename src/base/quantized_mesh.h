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

#ifndef EMODNET_QMGC_QUANTIZED_MESH_H
#define EMODNET_QMGC_QUANTIZED_MESH_H


#include <string>
#include <vector>
#include "cgal/cgal_utils.h"
#include <ctb.hpp>
#include "misc_utils.h"

/**
 * @class QuantizedMesh
 * @brief Contains all the data of a Quantized-mesh
 */
class QuantizedMesh {

public:
    // --- Various structs as described in the documentation ---
    // (https://cesiumjs.org/data-and-assets/terrain/formats/quantized-mesh-1.0.html)
    struct Header
    {
        // The center of the tile in Earth-centered Earth-Fixed coordinates
        double CenterX; //!< Center of the tile in Earth-centered Earth-Fixed, coordinate X
        double CenterY; //!< Center of the tile in Earth-centered Earth-Fixed, coordinate Y
        double CenterZ; //!< Center of the tile in Earth-centered Earth-Fixed, coordinate Z

        // The minimum and maximum heights in the area covered by this tile.
        // The minimum may be lower and the maximum may be higher than
        // the height of any vertex in this tile in the case that the min/max vertex
        // was removed during mesh simplification, but these are the appropriate
        // values to use for analysis or visualization.
        float MinimumHeight; //!< Minimum height possible for this tile
        float MaximumHeight; //!< Maximum height possible for this tile

        // The tile’s bounding sphere.  The X,Y,Z coordinates are again expressed
        // in Earth-centered Fixed coordinates, and the radius is in meters.
        double BoundingSphereCenterX; //!< Center of the bounding sphere of the tile in Earth-centered Earth-Fixed, coordinate X
        double BoundingSphereCenterY; //!< Center of the bounding sphere of the tile in Earth-centered Earth-Fixed, coordinate Y
        double BoundingSphereCenterZ; //!< Center of the bounding sphere of the tile in Earth-centered Earth-Fixed, coordinate Z
        double BoundingSphereRadius;  //!< Radius of the bounding sphere of the tile in meters

        // The horizon occlusion point, expressed in the ellipsoid-scaled Earth-centered Fixed frame.
        // If this point is below the horizon, the entire tile is below the horizon.
        // See http://cesiumjs.org/2013/04/25/Horizon-culling/ for more information.
        double HorizonOcclusionPointX; //!< X coordinate of the horizon occlusion point of the tile
        double HorizonOcclusionPointY; //!< Y coordinate of the horizon occlusion point of the tile
        double HorizonOcclusionPointZ; //!< Z coordinate of the horizon occlusion point of the tile
    };

    struct VertexData
    {
        unsigned int vertexCount = 0; //!< Number of vertices
        std::vector<unsigned short> u; //!< U coordinate of the vertex
        std::vector<unsigned short> v; //!< v coordinate of the vertex
        std::vector<unsigned short> height; //!< height coordinate of the vertex
    };

    struct IndexData
    {
        unsigned int triangleCount ; //!< Number of triangles in the tile
        std::vector< unsigned int > indices ; //!< Indices of the triangles in the tile, size triangleCount*3. While they can be ints or shorts, we store them as ints
    };

    struct EdgeIndices
    {
        unsigned int westVertexCount; //!< Number of vertices in the western border of the tile
        std::vector<unsigned int> westIndices; //!< The indices of the vertices falling in the western border of the tile

        unsigned int southVertexCount; //!< Number of vertices in the southern border of the tile
        std::vector<unsigned int> southIndices; //!< The indices of the vertices falling in the southern border of the tile

        unsigned int eastVertexCount; //!< Number of vertices in the eastern border of the tile
        std::vector<unsigned int> eastIndices; //!< The indices of the vertices falling in the eastern border of the tile

        unsigned int northVertexCount; //!< Number of vertices in the northern border of the tile
        std::vector<unsigned int> northIndices; //!< The indices of the vertices falling in the northern border of the tile
    };

    struct VertexNormals
    {
        std::vector<float> nx; //!< X coordinate of the normal
        std::vector<float> ny; //!< Y coordinate of the normal
        std::vector<float> nz; //!< Z coordinate of the normal
    };

    struct WaterMask {
        std::vector<unsigned char> mask ; //!< Water mask. Can be either a single value or 256x256
    };

    enum ExtensionIds {OCT_VERTEX_NORMALS = 1, WATER_MASK = 2};

    // --- Functions ---
    /// Constructor
    QuantizedMesh() {
        m_header = Header();
        m_vertexData = VertexData();
        m_indexData = IndexData();
        m_edgeIndices = EdgeIndices();
        m_vertexNormals = VertexNormals();
        m_waterMask = WaterMask();
        m_bytesPerIndex = 0;
    }

    /// Constructor from file
    QuantizedMesh(const std::string &filePath) { readFile(filePath); }

    /// Read the tile from a file
    bool readFile(const std::string &filePath);

    /// Write the tile to a file
    bool writeFile(const std::string &filePath);

    /// Show the contents of the tile on screen
    void print();

    /// Show the contents of the header of the tile on screen
    void printHeader();

    /// Set the header part of the quantized mesh structure
    void setHeader(const Header& header) { m_header = header; }

    /// Set the vertex data part of the quantized mesh structure
    void setVertexData(const VertexData& vd) { m_vertexData = vd; }

    /// Set the index data part of the quantized mesh structure
    void setIndexData(const IndexData& id) { m_indexData = id; }

    /// Set the edge indices data part of the quantized mesh structure
    void setEdgeIndices(const EdgeIndices& ei) { m_edgeIndices = ei; }

    /// Set the vertices normal part of the quantized mesh structure
    void setVertexNormals(const VertexNormals& vn) { m_vertexNormals = vn; }

    /// Get the header part of the quantized mesh structure
    Header getHeader() const { return m_header; }

    /// Get the vertex data part of the quantized mesh structure
    VertexData getVertexData() const { return m_vertexData; }

    /// Get the index data of the quantized mesh structure
    IndexData getIndexData() const { return m_indexData; }

    /// Get the vertex normals of the quantized mesh structure
    VertexNormals getVertexNormals() const { return m_vertexNormals; }

    /// Get the edge indices of the quantized mesh structure
    EdgeIndices getEdgeIndices() const { return m_edgeIndices; }

    // --- Constants ---
    const unsigned short int TILE_SIZE = 65;
    static const unsigned short MAX_VERTEX_DATA = 32767;

    // --- Convenience static functions ---
    /**
     * @brief Remap a value to be between 0 and 32767
     * @param value Value in the original range
     * @param minOr Minimum value in the original range
     * @param maxOr Maximum value in the original range
     * @return
     */
    static unsigned short remapToVertexDataValue(const double& value, const double& minOr, const double& maxOr) {
        double valueRemap = remap(value, minOr, maxOr, 0, MAX_VERTEX_DATA) ;
        return static_cast<unsigned short>(valueRemap);
    }

    static double remapFromVertexDataValue(const double& value, const double& minOr, const double& maxOr) {
        return remap(value, 0, MAX_VERTEX_DATA, minOr, maxOr ) ;
    }



private:

    // --- Attributes ---
    Header m_header ;
    VertexData m_vertexData ;
    IndexData m_indexData ;
    EdgeIndices m_edgeIndices ;
    int m_bytesPerIndex ; // Number of bytes used per index
    VertexNormals m_vertexNormals ;
    WaterMask m_waterMask ;

    // --- Functions ---

    // Decode a zig-zag encoded value
    unsigned short zigZagDecode( const unsigned short &value ) {
        return (value >> 1) ^ (-(value & 1)) ;
    }

    /// Encode a value using zig-zag encoding
    unsigned short zigZagEncode( const short &value ) {
        return (value << 1) ^ (value >> 31) ;
    }

    /// Encode a vector using oct-encoding
    void octEncode( const float xyz[3], unsigned char octxy[2] ) {
        const float invL1Norm = (1.0f) / (fabs(xyz[0]) + fabs(xyz[1]) + fabs(xyz[2]));

        if (xyz[2] < 0.0f) {
            octxy[0] = static_cast<unsigned char>((1.0f - float(fabs(xyz[1] * invL1Norm))) * sign(xyz[0]));
            octxy[1] = static_cast<unsigned char>((1.0f - float(fabs(xyz[0] * invL1Norm))) * sign(xyz[1]));
        } else {
            octxy[0] = static_cast<unsigned char>(xyz[0] * invL1Norm);
            octxy[1] = static_cast<unsigned char>(xyz[1] * invL1Norm);
        }
    }

    /// Decode a vector using oct-encoding
    void octDecode( const unsigned char octxy[2], float xyz[3] ) {

        xyz[0] = uchar2float(octxy[0]);
        xyz[1] = uchar2float(octxy[1]);
        xyz[2] = 1.0f - (fabs(xyz[0]) + fabs(xyz[1]));

        if (xyz[2] < 0.0f) {
            float oldX = xyz[0];
            xyz[0] = ((1.0f) - fabs(xyz[1])) * sign(oldX);
            xyz[1] = ((1.0f) - fabs(oldX))   * sign(xyz[1]);
        }
    }

    /// Returns -1 if negative, 1 otherwise
    inline float sign(float v) {
        return (v < 0.0f) ? -1.0f : 1.0f;
    }

    /// Decodes an unsigned char to a float (previously coded with float2uchar)
    float uchar2float( unsigned char u ) {
        return float(clamp(int(8) * (1.0f / float((uint64_t(1) << 7) - 1)), -1.0f, 1.0f));
    }

    /// Encodes a float with the 8 bits of the char
    unsigned char float2uchar( float f ) {
        return (unsigned char)round(clamp(f, -1.0f, 1.0f) * ((uint64_t(1) << 7) - 1));
    }

    /// Clamps a value between low and hi limits
    inline float clamp(float val, float low, float hi) {
        if (val <= low) {
            return low;
        } else if (val >= hi) {
            return hi;
        } else {
            return val;
        }
    }

    // Round a value
    inline float round(float f) {
        return floor(f + 0.5f);
    }

};


#endif //EMODNET_QMGC_QUANTIZED_MESH_H
