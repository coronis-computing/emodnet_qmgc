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

#include "quantized_mesh_tile.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include "cgal/cgal_utils.h"
#include <fstream>



void QuantizedMeshTile::convertUVHToLonLatHeight( const unsigned short &u, const unsigned short &v, const unsigned short &h,
                                                  double &lon, double &lat, double &height ) const
{
    const ctb::GlobalGeodetic profile;
    const ctb::CRSBounds tileBounds = profile.tileBounds(*this);
    const Header header = getHeader() ;

    lon = tileBounds.getMinX() + fabs( tileBounds.getMaxX() - tileBounds.getMinX() ) * (double)u/(double)QuantizedMesh::MAX_VERTEX_DATA ;
    lat = tileBounds.getMinY() + fabs( tileBounds.getMaxY() - tileBounds.getMinY() ) * (double)v/(double)QuantizedMesh::MAX_VERTEX_DATA ;
    height = header.MinimumHeight + fabs( header.MaximumHeight - header.MinimumHeight ) * (double)h/(double)QuantizedMesh::MAX_VERTEX_DATA ;
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



TinCreation::Point_3 QuantizedMeshTile::horizonOcclusionPoint( const std::vector<TinCreation::Point_3> &pts, const TinCreation::Point_3 &center )
{
    // Compute the scaledSpaceDirectionToPoint, a vector passing through the center of the ellipsoid and scaled on it
    // https://groups.google.com/forum/#!topic/cesium-dev/8NTW1Wl0d8s
    const double rX = m_ellipsoid.getRadiusX();
    const double rY = m_ellipsoid.getRadiusY();
    const double rZ = m_ellipsoid.getRadiusZ();

    TinCreation::Vector_3 scaledSpaceDirectionToPoint = Vector_3( center.x()/rX, center.y()/rY, center.z()/rZ ) ;
    scaledSpaceDirectionToPoint = scaledSpaceDirectionToPoint / sqrt(scaledSpaceDirectionToPoint.squared_length()) ; // Normalize

    std::vector< TinCreation::Point_3 >::const_iterator it ;
    double maxMagnitude = 0 ;
    for ( it = pts.begin(); it != pts.end(); ++it ) {
        double magnitude = computeHorizonOcclusionPointMagnitude( *it, scaledSpaceDirectionToPoint ) ;
        if ( magnitude > maxMagnitude )
            maxMagnitude = magnitude ;
    }

    TinCreation::Vector_3 hov = scaledSpaceDirectionToPoint * maxMagnitude ;

    return TinCreation::Point_3( hov.x(), hov.y(), hov.z() ) ;
}



double QuantizedMeshTile::computeHorizonOcclusionPointMagnitude( const TinCreation::Point_3 &position, const TinCreation::Vector_3 &scaledSpaceDirectionToPoint )
{
    const double rX = m_ellipsoid.getRadiusX();
    const double rY = m_ellipsoid.getRadiusY();
    const double rZ = m_ellipsoid.getRadiusZ();

    TinCreation::Vector_3 scaledSpaceVector( position.x()/rX, position.y()/rY, position.z()/rZ ) ;

    double magnitudeSquared = scaledSpaceVector.squared_length();
    double magnitude = sqrt(magnitudeSquared);
    TinCreation::Vector_3 unitScaledSpaceVector = scaledSpaceVector / magnitude ;

    // For the purpose of this computation, points below the ellipsoid
    // are considered to be on it instead.
    magnitudeSquared = std::max(1.0, magnitudeSquared);
    magnitude = std::max(1.0, magnitude);

    double cosAlpha = unitScaledSpaceVector * scaledSpaceDirectionToPoint ;
    double sinAlpha = sqrt( CGAL::cross_product( unitScaledSpaceVector, scaledSpaceDirectionToPoint ).squared_length() );
    double cosBeta = 1.0 / magnitude;
    double sinBeta = sqrt(magnitudeSquared - 1.0) * cosBeta ;

    return 1.0 / (cosAlpha * cosBeta - sinAlpha * sinBeta);
}