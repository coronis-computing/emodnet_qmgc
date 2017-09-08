//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
#define EMODNET_TOOLS_QUANTIZED_MESH_TILER_H

#include "quantized_mesh.h"
#include "quantized_mesh_tile.h"
#include "cgal_defines.h"
#include <ctb.hpp>



class QuantizedMeshTiler : public ctb::GDALTiler
{
public:

    /// Instantiate a tiler with all required arguments
    QuantizedMeshTiler(GDALDataset *poDataset, const ctb::Grid &grid, const ctb::TilerOptions &options):
        GDALTiler(poDataset, grid, options) {}

    /// Instantiate a tiler with an empty dataset
    QuantizedMeshTiler():
        GDALTiler() {}

    /// Instantiate a tiler with a dataset and grid but no options
    QuantizedMeshTiler(GDALDataset *poDataset, const ctb::Grid &grid):
            QuantizedMeshTiler(poDataset, grid, ctb::TilerOptions()) {}

    /// Overload the assignment operator
    QuantizedMeshTiler & operator=(const QuantizedMeshTiler &other);

    /// Override to return a covariant data type
    QuantizedMeshTile * createTile(const ctb::TileCoordinate &coord) const override;

private:

    /// Create a `GDALTile` representing the required terrain tile data (copied from ctb::TerrainTiler)
    virtual ctb::GDALTile *createRasterTile(const ctb::TileCoordinate &coord) const override;

    /**
     * @brief Get terrain bounds shifted to introduce a pixel overlap
     *
     * Given a `TileCoordinate`, this sets the resolution and returns latitude
     * and longitude bounds for a tile which include a pixel's worth of data
     * outside the actual tile bounds to both the east and the north.  This is
     * used to satisfy the quantized mesh specification of terrain tiles
     * sharing vertices at edges of surrounding tiles.
     *
     * @param coord The tile coordinate identifying the tile in question
     * @param resolution The resolution of the modified extent is set here
     */
    inline ctb::CRSBounds
    terrainTileBounds(const ctb::TileCoordinate &coord,
                      double &resolution) const {
        // The actual tile size accounting for a border
        ctb::i_tile lTileSize = mGrid.tileSize() - 1;
        ctb::CRSBounds tile = mGrid.tileBounds(coord); // the actual tile bounds

        // Get the resolution for the dataset without a border
        resolution = (tile.getMaxX() - tile.getMinX()) / lTileSize;

        // extend the easting by one pixel's worth
        tile.setMinX(tile.getMinX() - resolution);

        // extend the northing by one pixel's worth
        tile.setMaxY(tile.getMaxY() + resolution);

        return tile;
    }
};


#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
