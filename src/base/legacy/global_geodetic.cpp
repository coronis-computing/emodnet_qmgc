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

#include "global_geodetic.h"
#include <cmath>



GlobalGeodetic::GlobalGeodetic( bool tmsCompatible, int tileSize )
{
    m_tileSize = tileSize ;
    if ( tmsCompatible ) {
        m_resFact = 180.0 / m_tileSize ;
        m_numberOfLevelZeroTilesX = 2 ;
        m_numberOfLevelZeroTilesY = 1 ;
    }
    else {
        m_resFact = 360.0 / m_tileSize ;
        m_numberOfLevelZeroTilesX = 1 ;
        m_numberOfLevelZeroTilesY = 1 ;
    }
}



//! Converts lon/lat to pixel coordinates in given zoom of the EPSG:4326 pyramid
void GlobalGeodetic::lonLatToPixels( const double &lon, const double &lat, const int &zoom, double& px, double py )
{
    double res = m_resFact / pow( 2 , zoom ) ;
    px = (180 + lon) / res ;
    py = (90 + lat) / res ;
}



//! Returns coordinates of the tile covering region in pixel coordinates
void GlobalGeodetic::pixelsToTile( const double &px, const double &py, int tx, int ty )
{
    if ( px > 0 )
        tx = (int) ceil( px / m_tileSize ) - 1 ;
    else
        tx = 0 ;
    if ( py > 0 )
        ty = (int) ceil( py / m_tileSize ) - 1 ;
    else
        ty = 0 ;
}



//! Returns the tile for zoom which covers given lon/lat coordinates
void GlobalGeodetic::lonLatToTile(const double &lon, const double &lat, const int &zoom, int tx, int ty )
{
    double px, py ;
    lonLatToPixels( lon, lat, zoom, px, py ) ;
    pixelsToTile( px, py, tx, ty ) ;
}



//! Resolution (arc/pixel) for given zoom level (measured at Equator)
double GlobalGeodetic::resolution( const int &zoom )
{
    return m_resFact / pow( 2 , zoom ) ;
}



//! Maximal scaledown zoom of the pyramid closest to the pixelSize
int GlobalGeodetic::zoomForPixelSize( const double &pixelSize )
{
    for ( int i = 0; i < MAXZOOMLEVEL; i++ ) {
        if ( pixelSize > resolution(i) ) {
            if ( i != 0 )
                return i-1 ;
            else
                return 0 ;
        }
    }
}



//! Returns bounds of the given tile in the SWNE form
TileBounds GlobalGeodetic::tileLatLonBounds( const int &tx, const int &ty, const int& zoom )
{
    TileBounds bounds ;
    double res = resolution( zoom ) ;
    bounds.minLongitude = tx * m_tileSize * res - 180 ;
    bounds.maxLongitude = (tx + 1) * m_tileSize * res - 180 ;
    bounds.minLatitude = ty * m_tileSize * res - 90 ;
    bounds.maxLatitude = (ty + 1) * m_tileSize * res - 90 ;

    return bounds ;
}



//! Returns the number of tiles over x at a given zoom level (only 256px)
int GlobalGeodetic::getNumberOfXTilesAtZoom( const int &zoom )
{
    m_numberOfLevelZeroTilesX << zoom ;
}



//! Returns the number of tiles over y at a given zoom level (only 256px)
int GlobalGeodetic::getNumberOfYTilesAtZoom( const int &zoom )
{
    m_numberOfLevelZeroTilesY << zoom ;
}



