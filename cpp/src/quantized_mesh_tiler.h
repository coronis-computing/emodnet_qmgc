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
        bool IsBathymetry = false ;
        Ellipsoid RefEllipsoid = WGS84Ellipsoid() ;
        int HeighMapSamplingSteps = 256 ; // Maximum!
        int SimpStopEdgesCount = 128 ;
        double SimpWeightVolume = 0.5 ;
        double SimpWeightBoundary = 0.5 ;
        double SimpWeightShape = 0.0 ;
        float ClippingHighValue = std::numeric_limits<float>::infinity() ;
        float ClippingLowValue = -std::numeric_limits<float>::infinity() ;
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
        if ( m_options.HeighMapSamplingSteps < 0 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options HeighMapSamplingSteps < 0, defaulting to 256" << std::endl;
            m_options.HeighMapSamplingSteps = 256 ;
        }
        if ( m_options.HeighMapSamplingSteps > 256 ) {
            std::cerr << "[WARNING] QuantizedMeshTiler::options HeighMapSamplingSteps > 256 (maximum), defaulting to 256" << std::endl;
            m_options.HeighMapSamplingSteps = 256 ;
        }
//        if ( m_options.SimpCountRatioStop < 0 ) {
//            std::cerr << "[WARNING] QuantizedMeshTiler::options SimpCountRatioStop < 0 (should be between 0 and 1), defaulting to 0.05" << std::endl;
//            m_options.SimpCountRatioStop = 0.05 ;
//        }
//        if ( m_options.SimpCountRatioStop > 1 ) {
//            std::cerr << "[WARNING] QuantizedMeshTiler::options SimpCountRatioStop > 1 (should be between 0 and 1), defaulting to 0.05" << std::endl;
//            m_options.SimpCountRatioStop = 0.05 ;
//        }
//        if ( (m_options.SimpVolumeWeight + m_options.SimpBoundaryWeight + m_options.SimpShapeWeight) > 1 )

    }

    float clip(const float& n, const float& lower, const float& upper) const {
        return std::max(lower, std::min(n, upper));
    }
};

#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
