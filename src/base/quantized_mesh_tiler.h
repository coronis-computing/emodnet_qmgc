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

#ifndef EMODNET_QMGC_QUANTIZED_MESH_TILER_H
#define EMODNET_QMGC_QUANTIZED_MESH_TILER_H

#include "quantized_mesh.h"
#include "quantized_mesh_tile.h"
#include "tin_creation/tin_creation_cgal_types.h"
#include "tiling/raster_tiler.h"
#include <boost/filesystem.hpp>
#include "ellipsoid.h"
#include "tin_creation/tin_creator.h"
#include "borders_data.h"

namespace fs = boost::filesystem ;


/**
 * @class QuantizedMeshTiler
 * @brief Tiler of a terrain generating quantized-mesh tiles
 *
 * @note Holds a qmgc::RasterTiler rather than deriving from one. Deriving from
 *       cesium-terrain-builder's TerrainTiler was what dragged the heightmap
 *       format's one-sample border into this tiler; composition also lets the
 *       compiler generate the copy constructor, which previously had to be
 *       written out by hand to dodge a non-copyable mutex.
 */
class QuantizedMeshTiler
{
    // --- Private Typedefs ---
    typedef TinCreation::Point_3 Point_3;
    typedef TinCreation::Vector_3 Vector_3;
    typedef TinCreation::Point_2 Point_2;
    typedef TinCreation::Polyhedron Polyhedron;

public:
    // --- Options struct ---
    struct QMTOptions {
        bool IsBathymetry = false ;                     //!< Flag indicating wether the values on the raster must be considered as elevations (false) or depths (true)
        Ellipsoid RefEllipsoid = WGS84Ellipsoid() ;     //!< The reference ellipsoid of the tile (needed to compute horizon occlusion point)
        int HeighMapSamplingSteps = 256 ;               //!< Sampling steps. Maximum = 256!
        float ClippingHighValue = std::numeric_limits<float>::infinity() ; //!< Maximum value allowed on the raster, clip values if larger
        float ClippingLowValue = -std::numeric_limits<float>::infinity() ; //!< Minimum value allowed on the raster, clip values if smaller
        float AboveSeaLevelScaleFactor = -1;    //!< Scale factor to apply to the readings above sea level (ignored if < 0)
        float BelowSeaLevelScaleFactor = -1;    //!< Scale factor to apply to the readings below sea level (ignored if < 0)
    };

    // --- Methods ---

    /**
     * @brief Constructor: instantiates a tiler with all required arguments
     *
     */
    QuantizedMeshTiler(GDALDataset *dataset,
                       const qmgc::GlobalGeodeticGrid &grid,
                       const qmgc::TilerOptions &tilerOptions,
                       const QMTOptions& options,
                       const TinCreation::TinCreator& tinCreator )
            : m_rasterTiler(dataset, grid, tilerOptions)
            , m_options(options)
            , m_tinCreator(tinCreator)
            {checkOptions();}

    /// Get the underlying raster tiler
    const qmgc::RasterTiler& rasterTiler() const { return m_rasterTiler; }

    /// Get the tiling grid
    const qmgc::GlobalGeodeticGrid& grid() const { return m_rasterTiler.grid(); }

    /// Get the source dataset's extent, in EPSG:4326 degrees
    const qmgc::CRSBounds& bounds() const { return m_rasterTiler.bounds(); }

    /// Get the deepest zoom level justified by the source dataset's resolution
    qmgc::i_zoom maxZoomLevel() const { return m_rasterTiler.maxZoomLevel(); }

    /// Get the tiles spanned by the source dataset at a given zoom level
    qmgc::TileBounds tileBoundsForZoom(qmgc::i_zoom zoom) const {
        return m_rasterTiler.tileBoundsForZoom(zoom);
    }

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
     * @param coord TileCoordinate.
     * @param bd Data to preserve for the borders.
     *
     * @return The quantized mesh tile.
     */
    QuantizedMeshTile createTile(const qmgc::TileCoordinate &coord, BordersData& bd) ;

    /**
     * @brief Get the creation options for this tiler (QMTOptions structure)
     * @return QMTOptions structure
     */
    QMTOptions getOptions() { return m_options; }

    /**
     * Sets the TIN creator parameter related with the current zoom level.
     * @param zoom The zoom level
     */
    void setTinCreatorParamsForZoom(const unsigned int& zoom) { m_tinCreator.setParamsForZoom(zoom); }

    /**
     * @brief Get the heightmap values from the GDAL raster in normalized coordinates
     *
     * @param coord The coordinates of the tile
     * @param bd Data falling in the borders of the tile. Note: this is not const because border vertices are input as tile coordinates, but are output in uvh coordinates
     * @param[out] minHeight Min height on the tile from raster
     * @param[out] maxHeight Max height on the tile from raster
     * @param[out] tileBounds Output variable containing the tile bounds
     * @return Vector of points in the heightmap
     */
    std::vector<Point_3> getUVHPointsFromRaster(const qmgc::TileCoordinate &coord,
                                                BordersData& bd,
                                                float& minHeight, float& maxHeight,
                                                qmgc::CRSBounds& tileBounds) const ;

private:
    // --- Attributes ---
    qmgc::RasterTiler m_rasterTiler; //!< Supplies the raster covered by each tile
    QMTOptions m_options;
    TinCreation::TinCreator m_tinCreator;

    // --- Private Functions ---
    /**
     * @brief Ensure that the options passed are valid (warning raised and defaults set otherwise)
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

    /**
     * @brief Clip a value between lower and upper bounds.
     * @param n The number to clip.
     * @param lower Clipping lower bound.
     * @param upper Clipping upper bound.
     * @return Clipped value.
     */
    float clip(const float& n, const float& lower, const float& upper) const {
        return std::max(lower, std::min(n, upper));
    }

    // --- The following private functions split the processing required to generate the tiles for better readability ---



    /**
     * @brief Compute the values of the header from the points in the simplified TIN
     *
     * @param qmTile The tile where the header will be written
     * @param surface The triangle mesh containing the geometry of the tile (only its vertices will be used here)
     * @param minHeight Min height on the tile from raster
     * @param maxHeight Max height on the tile from raster
     * @param tileBounds The tile bounds (in the geographic reference system coordinates)
     */
    void computeQuantizedMeshHeader(QuantizedMeshTile& qmTile,
                                    const Polyhedron& surface,
                                    const float& minHeight, float& maxHeight,
                                    const qmgc::CRSBounds& tileBounds) const ;


    /**
     * Compute and store all the parts of the Quantized Mesh format related to the geometry of the TIN
     * While computing this information, we also store the vertices in the borders of the tile
     *
     * @param qmTile The QuantizedMeshTile structure to modify
     * @param surface The base triangle mesh containing the (simplified) geometry of the tile
     * @param minHeight Min height on the tile from raster
     * @param maxHeight Max height on the tile from raster
     * @param[out] tileEastVertices Eastern vertices of the tile
     * @param[out] tileWestVertices Western vertices of the tile
     * @param[out] tileNorthVertices Northern vertices of the tile
     * @param[out] tileSouthVertices Southern vertices of the tile
     */
    void computeQuantizedMeshGeometry(QuantizedMeshTile& qmTile,
                                      Polyhedron& surface,
                                      const float& minHeight, const float& maxHeight,
                                      std::vector<Point_3> &tileEastVertices,
                                      std::vector<Point_3> &tileWestVertices,
                                      std::vector<Point_3> &tileNorthVertices,
                                      std::vector<Point_3> &tileSouthVertices ) const ;
};

#endif //EMODNET_QMGC_QUANTIZED_MESH_TILER_H
