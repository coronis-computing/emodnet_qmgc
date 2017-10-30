//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
#define EMODNET_TOOLS_QUANTIZED_MESH_TILER_H

#include "quantized_mesh.h"
#include "quantized_mesh_tile.h"
#include "cgal_defines.h"
#include <ctb.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem ;




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

    QuantizedMeshTile* createTile(const ctb::TileCoordinate &coord) const override { return new QuantizedMeshTile(coord) ;} // Dummy

    /**
     * \brief Create the quantized mesh tile.
     *
     * Create the quantized-mesh tile given the raster enclosed by the coordinates \p coord and the vertices to maintain from previous tiles
     * The original full resolution regular grid extracted from the rasters is decimated to obrain a Triangulated Irregular Network (TIN)
     *
     * Note: the parameters tileEastVertices and tileNorthVertices represent the vertices to maintain from the neighboring tiles on input,
     * but after the function they are output parameters containing the eastern/northen vertices to maintain for the CURRENT tile
     *
     */
    QuantizedMeshTile* createTile(const ctb::TileCoordinate &coord,
                                  std::vector<Point_3> &tileWestVertices,
                                  std::vector<Point_3> &tileSouthVertices ) const ;

    QuantizedMeshTile* createTileNoSimp(const ctb::TileCoordinate &coord ) const ;



    /**
     * \brief Create the tile pyramid
     *
     * Due to the quantized-mesh format requiring the vertices on the edges to coincide between neighbors, the creation
     * of the tiles' for each zoom is not as simple as in the heightmap format, and it requires a more complex loop taking
     * into account vertices at borders of the neighbors of the current tile
     */
    void createTilePyramid(const int &startZoom, const int &endZoom, const std::string &outDir) ;

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

    static std::string getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                 const std::string &mainOutDir )
    {
        // Check/create the tile folder (zoom/x)
        fs::path mainOutDirPath(mainOutDir) ;
        fs::path tileFolder = mainOutDirPath / fs::path(std::to_string(coord.zoom)) / fs::path(std::to_string(coord.x)) ;
        if ( !fs::exists( tileFolder ) && !fs::create_directories( tileFolder ) ) {
            std::cerr << "[ERROR] Cannot create the tile folder" << tileFolder << std::endl ;
            return std::string() ;
        }

        fs::path fileNamePath = tileFolder / fs::path(std::to_string(coord.y) + ".terrain") ;

        return fileNamePath.string() ;
    }
};


#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
