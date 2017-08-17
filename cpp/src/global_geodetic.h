//
// Created by Ricard Campos
// Reimplementation in C++ of the global_geodetic.py file from quantized_mesh_tile project
// https://github.com/loicgasser/quantized-mesh-tile/blob/master/quantized_mesh_tile/global_geodetic.py
//

#ifndef EMODNET_TOOLS_GLOBAL_GEODETIC_H
#define EMODNET_TOOLS_GLOBAL_GEODETIC_H


struct TileBounds {
    double minLatitude ;
    double minLongitude ;
    double maxLatitude ;
    double maxLongitude ;
};


class GlobalGeodetic {

public:

    GlobalGeodetic( bool tmsCompatible = true, int tileSize = 256 ) ;

    //! Converts lon/lat to pixel coordinates in given zoom of the EPSG:4326 pyramid
    void latLonToPixels( const double &lat, const double &lon, const int &zoom, double& px, double py ) ;

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
    int GetNumberOfXTilesAtZoom( const int &zoom ) ;

    //! Returns the number of tiles over y at a given zoom level (only 256px)
    int GetNumberOfYTilesAtZoom( const int &zoom ) ;

private:
    int m_tileSize ;
    int m_numberOfLevelZeroTilesX ;
    int m_numberOfLevelZeroTilesY ;

};


#endif //EMODNET_TOOLS_GLOBAL_GEODETIC_H
