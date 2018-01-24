//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
#define EMODNET_TOOLS_QUANTIZED_MESH_TILER_H

#include "quantized_mesh.h"
#include "quantized_mesh_tile.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include <ctb.hpp>
#include <boost/filesystem.hpp>
#include "ellipsoid.h"
#include "tin_creation/tin_creator.h"
#include <mutex>

namespace fs = boost::filesystem ;



class QuantizedMeshTiler : public ctb::TerrainTiler
{
    // --- Private Typedefs ---
    typedef TinCreation::Point_3 Point_3;
    typedef TinCreation::Vector_3 Vector_3;
    typedef TinCreation::Point_2 Point_2;
    typedef TinCreation::Polyhedron Polyhedron;

public:
    // --- Options struct ---
    struct QMTOptions {
        bool IsBathymetry = false ;                     // Flag indicating wether the values on the raster must be considered as elevations (false) or depths (true)
        Ellipsoid RefEllipsoid = WGS84Ellipsoid() ;     // The reference ellipsoid of the tile (needed to compute horizon occlusion point)
        int HeighMapSamplingSteps = 256 ;               // Maximum!
        float ClippingHighValue = std::numeric_limits<float>::infinity() ; // Maximum value allowed on the raster, clip values if larger
        float ClippingLowValue = -std::numeric_limits<float>::infinity() ; // Minimum value allowed on the raster, clip values if smaller
    };

    // --- Methods ---

    /// Constructor: instantiates a tiler with all required arguments
    // Note: We use a string instead of a GDALDataset*, as in ctb, because we want a new GDALDataset* to be opened for each tiler (they are not threadsafe!)
    QuantizedMeshTiler(GDALDataset *dataset,
                       const ctb::Grid &grid,
                       const ctb::TilerOptions &tilerOptions,
                       const QMTOptions& options,
                       const TinCreation::TinCreator& tinCreator )
            : TerrainTiler(dataset, grid, tilerOptions)
            , m_options(options)
            , m_tinCreator(tinCreator)
            {checkOptions();}

    /// A copy constructor that does not try to copy the mutex (needed because mutex are non-copyable)
    QuantizedMeshTiler(const QuantizedMeshTiler& tiler )
            : TerrainTiler(tiler.poDataset, tiler.mGrid, tiler.options)
            , m_options(tiler.m_options)
            , m_tinCreator(tiler.m_tinCreator)
    {}

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
    QuantizedMeshTile createTile(const ctb::TileCoordinate &coord,
                                  std::vector<Point_3> &tileEastVertices,
                                  std::vector<Point_3> &tileWestVertices,
                                  std::vector<Point_3> &tileNorthVertices,
                                  std::vector<Point_3> &tileSouthVertices) ;

private:
    // --- Attributes ---
    QMTOptions m_options ;
    TinCreation::TinCreator m_tinCreator ;
    mutable std::mutex m_mutex; // Mark mutex as mutable because it doesn't represent the object's real state
                                // Note that we don't need the mutex if we create multiple instances of tilers, as done in qm_tiler right now. We leave it here in case it is needed for other implementations

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
    }

    float clip(const float& n, const float& lower, const float& upper) const {
        return std::max(lower, std::min(n, upper));
    }

    // --- The following private functions split the processing required to generate the tiles for better readability ---

    /**
     * Get the heightmap values from the GDAL raster in normalized coordinates
     *
     * @param coord
     * @param tileEastVertices
     * @param tileWestVertices
     * @param tileNorthVertices
     * @param tileSouthVertices
     * @param[out] minHeight Min height on the tile from raster
     * @param[out] maxHeight Max height on the tile from raster
     * @param[out] tileBounds Output variable containing the tile bounds
     * @return vector of points in the heightmap
     */
    std::vector<Point_3> getUVHPointsFromRaster(const ctb::TileCoordinate &coord,
                                                const std::vector<Point_3> &tileEastVertices,
                                                const std::vector<Point_3> &tileWestVertices,
                                                const std::vector<Point_3> &tileNorthVertices,
                                                const std::vector<Point_3> &tileSouthVertices,
                                                float& minHeight, float& maxHeight,
                                                ctb::CRSBounds& tileBounds) const ;

    /**
     * Simplifies the surface using Lindstrom-Turk algorithm [1][2]
     * [1] Peter Lindstrom and Greg Turk. Fast and memory efficient polygonal simplification. In IEEE Visualization, pages 279–286, 1998.
     * [2] P. Lindstrom and G. Turk. Evaluation of memoryless simplification. IEEE Transactions on Visualization and Computer Graphics, 5(2):98–115, slash 1999.
     *
     * @param surface
     */
//    void simplifySurface( Polyhedron& surface,
//                          const bool& constrainEasternVertices,
//                          const bool& constrainWesternVertices,
//                          const bool& constrainNorthernVertices,
//                          const bool& constrainSouthernVertices ) const ;

    /**
     * Compute the values of the header from the points in the simplified TIN
     *
     * @param qmTile The tile where the header will be written
     * @param surface The triangle mesh containing the geometry of the tile (only its vertices will be used here)
     * @param tileBounds The tile bounds (in the geographic reference system coordinates)
     */
    void computeQuantizedMeshHeader(QuantizedMeshTile& qmTile,
                                    const Polyhedron& surface,
                                    const float& minHeight, float& maxHeight,
                                    const ctb::CRSBounds& tileBounds) const ;


    /**
     * Compute and store all the parts of the Quantized Mesh format related to the geometry of the TIN
     * While computing this information, we also store the vertices in the borders of the tile
     *
     * @param qmTile
     * @param surface
     * @param minHeight
     * @param maxHeight
     * @param tileEastVertices Eastern vertices of the tile
     * @param tileWestVertices Western vertices of the tile
     * @param tileNorthVertices Northern vertices of the tile
     * @param tileSouthVertices Southern vertices of the tile
     */
    void computeQuantizedMeshGeometry(QuantizedMeshTile& qmTile,
                                      Polyhedron& surface,
                                      const float& minHeight, const float& maxHeight,
                                      std::vector<Point_3> &tileEastVertices,
                                      std::vector<Point_3> &tileWestVertices,
                                      std::vector<Point_3> &tileNorthVertices,
                                      std::vector<Point_3> &tileSouthVertices ) const ;
};

#endif //EMODNET_TOOLS_QUANTIZED_MESH_TILER_H
