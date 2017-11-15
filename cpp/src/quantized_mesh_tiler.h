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
#include "ellipsoid.h"

namespace fs = boost::filesystem ;



class QuantizedMeshTiler : public ctb::GDALTiler
{
public:

    /// Constructor: instantiates a tiler with all required arguments
    QuantizedMeshTiler(GDALDataset *poDataset,
                       const ctb::Grid &grid,
                       const ctb::TilerOptions &options,
                       const bool& bathymetryFlag = false,
                       const Ellipsoid& e = WGS84Ellipsoid(),
                       const double& simpCountRatioStop = 0.05 )
            : GDALTiler(poDataset, grid, options)
            , m_isBathymetry(bathymetryFlag)
            , m_ellipsoid(e)
            , m_simpCountRatioStop(simpCountRatioStop) {}

    /// Default constructor: instantiates a tiler with an empty dataset and default settings
    QuantizedMeshTiler()
            : GDALTiler()
            , m_isBathymetry(false)
            , m_ellipsoid(WGS84Ellipsoid())
            , m_simpCountRatioStop(0.05) {}

//    /// Instantiate a tiler with a dataset and grid but no options
//    QuantizedMeshTiler(GDALDataset *poDataset,
//                       const ctb::Grid &grid,
//                       const bool& bathymetryFlag = false,
//                       const Ellipsoid& e = WGS84Ellipsoid(),
//                       const double& simpCountRatioStop )
//            : QuantizedMeshTiler(poDataset, grid, ctb::TilerOptions(), bathymetryFlag, e, simpCountRatioStop ) {}

    /// Overload the assignment operator
//    QuantizedMeshTiler & operator=(const QuantizedMeshTiler &other);

    // Dummy, we need it because we inherit from ctb::GDALTiler, but we do not use it!
    QuantizedMeshTile* createTile(const ctb::TileCoordinate &coord) const override { return new QuantizedMeshTile(coord) ;}

    /// Set bathymetry mode
    void setBathymetryFlag( bool isBathymetry ) { m_isBathymetry = isBathymetry ; }

    /**
     * @brief Create the quantized mesh tile.
     *
     * Create the quantized-mesh tile given the raster enclosed by the coordinates \p coord and the vertices to maintain from previous tiles
     * The original full resolution regular grid extracted from the rasters is decimated to obrain a Triangulated Irregular Network (TIN)
     *
     * Note: the parameters tileEastVertices and tileNorthVertices represent the vertices to maintain from the neighboring tiles on input,
     * but after the function they are output parameters containing the eastern/northen vertices to maintain for the CURRENT tile
     * Take into account that their 3D coordinates represent (u,v), in tile coordinates [0..QuantizedMesh::MAX_VERTEX_DATA], and height, where
     * this height is the raster-extracted height in meters.
     *
     */
    QuantizedMeshTile* createTile(const ctb::TileCoordinate &coord,
                                  std::vector<Point_3> &tileWestVertices,
                                  std::vector<Point_3> &tileSouthVertices ) const ;

    QuantizedMeshTile* createTileNoSimp(const ctb::TileCoordinate &coord ) const ;

    /**
     * @brief Creates the tile pyramid in quantized-mesh format
     *
     * Due to the quantized-mesh format requiring the vertices on the edges to coincide between neighbors, the creation
     * of the tiles' for each zoom is not as simple as in the heightmap format, and it requires a more complex loop taking
     * into account vertices at borders of the neighbors of the current tile being processed
     */
    void createTilePyramid(const int &startZoom, const int &endZoom, const std::string &outDir) ;

private:

    /// Create a `GDALTile` representing the required terrain tile data (same as ctb::TerrainTiler)
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

    /**
     * @brief Check that the tile folder (zoom/x) exists, and creates it otherwise.
     *
     * @return The path of the tile's file.
     */
    static std::string getTileFileAndCreateDirs( const ctb::TileCoordinate &coord,
                                                 const std::string &mainOutDir ) ;


    // --- Attributes ---
    bool m_isBathymetry ;
    Ellipsoid m_ellipsoid ;
    double m_simpCountRatioStop ;
};

#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
