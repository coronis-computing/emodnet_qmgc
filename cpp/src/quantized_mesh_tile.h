//
// Created by Ricard Campos
//

#ifndef EMODNET_TOOLS_QUANTIZEDMESHTILE_H
#define EMODNET_TOOLS_QUANTIZEDMESHTILE_H

#include "quantized_mesh.h"


class QuantizedMeshTile :
public QuantizedMesh, public ctb::Tile
{
    friend class TerrainTiler;

public:

    /// Create a quantized mesh tile from a tile coordinate
    QuantizedMeshTile(const ctb::TileCoordinate &coord);

    /// Create a quantized mesh tile from a file
    QuantizedMeshTile(const char *fileName, const ctb::TileCoordinate &coord);

    /// Create a quantized mesh tile from quantized mesh data
    QuantizedMeshTile(const QuantizedMesh &qm, const ctb::TileCoordinate &coord);

    //! Convert U/V/Height coordinates in quantized mesh tile to Lon/Lat/Height
    void convertUVHToLonLatHeight( const unsigned short &u, const unsigned short &v, const unsigned short &h,
                                   double &lon, double &lat, double &height ) ;

    //! Export to OFF format
    bool exportToOFF( const std::string &outFilePath, const bool& useRealWorldValues = true ) ;

    //! Creates a quantized-mesh-tile from a terrain tile
    bool convertFromHeightMapTile( const std::string &filePath ) ;
};



#endif //EMODNET_TOOLS_QUANTIZEDMESHTILE_H
