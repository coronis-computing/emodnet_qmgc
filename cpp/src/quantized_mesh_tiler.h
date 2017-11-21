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



class QuantizedMeshTiler : public ctb::TerrainTiler
{
public:

    struct QMTOptions {
        bool isBathymetry = false ;
        Ellipsoid ellipsoid = WGS84Ellipsoid() ;
        int heighMapSamples = 256 ; // Maximum!
        double simpCountRatioStop = 0.05 ;
    };

    /// Constructor: instantiates a tiler with all required arguments
    QuantizedMeshTiler(GDALDataset *poDataset,
                       const ctb::Grid &grid,
                       const ctb::TilerOptions &tilerOptions,
                       const QMTOptions& options )
            : TerrainTiler(poDataset, grid, tilerOptions)
            , m_options(options) {checkOptions();}

    /// Default constructor: instantiates a tiler with an empty dataset and default settings
    QuantizedMeshTiler()
            : TerrainTiler()
            , m_options() {}

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
                                  std::vector<Point_3> &tileEastVertices,
                                  std::vector<Point_3> &tileWestVertices,
                                  std::vector<Point_3> &tileNorthVertices,
                                  std::vector<Point_3> &tileSouthVertices) const ;

    QuantizedMeshTile* createTileNoSimp(const ctb::TileCoordinate &coord ) const ;

private:
    // --- Attributes ---
    QMTOptions m_options ;
//    bool m_isBathymetry ;
//    Ellipsoid m_ellipsoid ;
//    double m_simpCountRatioStop ;

    // --- Private Functions ---
    /**
     * Ensure that the options passed are valid (warning raised and defaults set otherwise)
     */
    void checkOptions() {
        if ( m_options.heighMapSamples < 0 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options heighMapSamples < 0, defaulting to 256" << std::endl;
            m_options.heighMapSamples = 256 ;
        }
        if ( m_options.heighMapSamples > 256 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options heighMapSamples > 256 (maximum), defaulting to 256" << std::endl;
            m_options.heighMapSamples = 256 ;
        }
        if ( m_options.simpCountRatioStop < 0 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options simpCountRatioStop < 0 (should be between 0 and 1), defaulting to 0.05" << std::endl;
            m_options.simpCountRatioStop = 0.05 ;
        }
        if ( m_options.simpCountRatioStop > 1 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options simpCountRatioStop > 1 (should be between 0 and 1), defaulting to 0.05" << std::endl;
            m_options.simpCountRatioStop = 0.05 ;
        }
    }
};

#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
