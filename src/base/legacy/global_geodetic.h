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

//
// Reimplementation in C++ of the global_geodetic.py file from quantized_mesh_tile project
// https://github.com/loicgasser/quantized-mesh-tile/blob/master/quantized_mesh_tile/global_geodetic.py
//

#ifndef EMODNET_QMGC_GLOBAL_GEODETIC_H
#define EMODNET_QMGC_GLOBAL_GEODETIC_H

#define MAXZOOMLEVEL 32

struct TileBounds {
    double minLatitude ;
    double minLongitude ;
    double maxLatitude ;
    double maxLongitude ;

    TileBounds() {
        minLatitude = -1 ;
        minLongitude = -1 ;
        maxLatitude = -1 ;
        maxLongitude = -1 ;
    }

};


class GlobalGeodetic {

public:

    GlobalGeodetic( bool tmsCompatible = true, int tileSize = 256 ) ;

    //! Converts lon/lat to pixel coordinates in given zoom of the EPSG:4326 pyramid
    void lonLatToPixels( const double &lon, const double &lat, const int &zoom, double& px, double py ) ;

    //! Returns coordinates of the tile covering region in pixel coordinates
    void pixelsToTile( const double &px, const double &py, int tx, int ty ) ;

    //! Returns the tile for zoom which covers given lon/lat coordinates
    void lonLatToTile(const double &lon, const double &lat, const int &zoom, int tx, int ty ) ;

    //! Resolution (arc/pixel) for given zoom level (measured at Equator)
    double resolution( const int &zoom ) ;

    //! Maximal scaledown zoom of the pyramid closest to the pixelSize
    int zoomForPixelSize( const double &pixelSize ) ;

    //! Returns bounds of the given tile in the SWNE form
    TileBounds tileLatLonBounds( const int &tx, const int &ty, const int& zoom ) ;

    //! Returns the number of tiles over x at a given zoom level (only 256px)
    int getNumberOfXTilesAtZoom( const int &zoom ) ;

    //! Returns the number of tiles over y at a given zoom level (only 256px)
    int getNumberOfYTilesAtZoom( const int &zoom ) ;

private:
    int m_tileSize ;
    double m_resFact ;
    int m_numberOfLevelZeroTilesX ;
    int m_numberOfLevelZeroTilesY ;

};


#endif //EMODNET_QMGC_GLOBAL_GEODETIC_H
