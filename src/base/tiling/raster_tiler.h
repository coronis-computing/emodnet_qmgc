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

#ifndef EMODNET_QMGC_TILING_RASTER_TILER_H
#define EMODNET_QMGC_TILING_RASTER_TILER_H

#include "geo_types.h"
#include "global_geodetic_grid.h"
#include "qmgc_exception.h"
#include <gdal_priv.h>
#include <memory>
#include <string>

namespace qmgc {

/// Deleter letting a GDALDataset be held by a std::unique_ptr
struct GdalDatasetDeleter {
    void operator()(GDALDataset *dataset) const {
        if (dataset != nullptr)
            GDALClose(static_cast<GDALDatasetH>(dataset));
    }
};

/// An owning handle to a GDALDataset
typedef std::unique_ptr<GDALDataset, GdalDatasetDeleter> GdalDatasetPtr;

/**
 * @brief Where a tile's samples are taken relative to the tile's extent.
 */
enum class SamplingMode {
    /**
     * @brief Sample centres land exactly on the tile's node grid.
     *
     * The first and last sample of each row/column coincide with the tile's own
     * edges, so a tile and its neighbour sample the shared edge at the identical
     * physical location. This is what the quantized-mesh format wants.
     */
    NodeRegistered,

    /**
     * @brief Reproduce cesium-terrain-builder's TerrainTiler window.
     *
     * CTB shifts the sampling window one whole sample west and north, because
     * the *heightmap* format requires a sample of overlap from the neighbouring
     * tiles. Quantized-mesh has no such requirement, and the shift leaves the
     * surface displaced by half a sample east and south relative to its true
     * coordinates. Retained only to reproduce pre-existing output bit for bit.
     */
    HeightmapBorder
};

/// Options controlling how a raster is warped onto the tile grid
struct TilerOptions {
    /// Resampling algorithm, as accepted by `gdalwarp -r`
    std::string resampleAlg = "average";

    /// Let GDAL pick a source overview matching the tile resolution (`-ovr AUTO`)
    bool useOverviews = true;

    /// Error threshold in pixels for the approximating transformer (`-et`).
    /// The gdalwarp default, 0.125, trades up to an eighth of a sample of
    /// geometric error for speed. Set to 0 to transform every point exactly.
    double errorThreshold = 0.125;

    /// Warp threads *per tile*. Keep at 1 when tiles are generated in parallel,
    /// otherwise the two levels of parallelism oversubscribe the machine.
    int warpThreads = 1;

    /// Where samples are taken relative to the tile extent
    SamplingMode samplingMode = SamplingMode::NodeRegistered;

    /// Declare the band's nodata value to the warper, so that resampling does
    /// not average nodata into neighbouring valid samples.
    bool excludeNoDataFromResampling = true;
};

/**
 * @brief The geometry of the raster window backing one tile.
 */
struct TileWindow {
    /// Extent actually warped, i.e. the outer edges of the sampled raster
    CRSBounds warpExtent;

    /// Extent the samples should be interpreted as spanning, which is what the
    /// quantized-mesh header's lat/lon must be derived from
    CRSBounds sampleExtent;

    /// Size of one sample, in degrees
    double resolution;
};

/**
 * @brief Compute the raster window backing a tile.
 *
 * Pure geometry, deliberately free of GDAL so that it can be unit tested on its own.
 *
 * For a tile spanning [minX, maxX] sampled at `numSamples` points, the spacing
 * between samples is `width / (numSamples - 1)` in both modes; the modes differ
 * only in where that run of samples is placed:
 *
 * - NodeRegistered shifts the window out by half a sample on every side, so the
 *   `numSamples` sample centres sit at `minX + i*res` for i in [0, numSamples-1],
 *   hitting both tile edges exactly.
 * - HeightmapBorder shifts the window a whole sample west and north, so sample 0
 *   falls outside the tile and sample `numSamples-1` falls half a sample inside it.
 *
 * @param grid The tiling grid
 * @param coord The tile in question
 * @param mode Where samples are taken relative to the tile extent
 * @param numSamples Samples along each side of the tile (at least 2)
 * @return The window geometry
 */
inline TileWindow computeTileWindow(const GlobalGeodeticGrid &grid,
                                    const TileCoordinate &coord,
                                    SamplingMode mode,
                                    int numSamples) {
    if (numSamples < 2)
        throw QmgcException("A tile must be sampled at least twice along each side");

    const CRSBounds tileBounds = grid.tileBounds(coord);

    // Sample spacing: `numSamples` samples spanning the tile inclusive of both
    // edges leaves `numSamples - 1` intervals between them.
    const double res = tileBounds.getWidth() / static_cast<double>(numSamples - 1);

    TileWindow window;
    window.resolution = res;

    if (mode == SamplingMode::NodeRegistered) {
        window.warpExtent = CRSBounds(tileBounds.getMinX() - (res / 2.0),
                                      tileBounds.getMinY() - (res / 2.0),
                                      tileBounds.getMaxX() + (res / 2.0),
                                      tileBounds.getMaxY() + (res / 2.0));
        window.sampleExtent = tileBounds;
    } else {
        // Reproduce ctb::TerrainTiler::terrainTileBounds
        window.warpExtent = CRSBounds(tileBounds.getMinX() - res,
                                      tileBounds.getMinY(),
                                      tileBounds.getMaxX(),
                                      tileBounds.getMaxY() + res);
        window.sampleExtent = window.warpExtent;
    }

    return window;
}

/**
 * @class RasterTiler
 * @brief Warps windows of a GDAL raster onto the TMS global geodetic grid.
 *
 * Replaces cesium-terrain-builder's GDALTiler/TerrainTiler pair. The warping
 * itself is delegated to GDAL's own `GDALWarp()` (the engine behind the
 * `gdalwarp` binary) rather than being assembled by hand from transformers and
 * warped VRTs, which is both far less code and keeps overview selection,
 * nodata handling and reprojection as GDAL's responsibility.
 *
 * @note The dataset is borrowed, not owned: the caller must keep it alive for
 *       as long as the tiler, and for as long as any copy of the tiler.
 *       Instances are cheap to copy, which is how one tiler per worker thread
 *       is set up.
 */
class RasterTiler {
public:
    /**
     * @brief Constructor
     * @param dataset The source raster. Borrowed; must outlive this object.
     * @param grid The tiling grid
     * @param options Warping options
     */
    RasterTiler(GDALDataset *dataset,
                const GlobalGeodeticGrid &grid,
                const TilerOptions &options);

    /// Get the source dataset's extent, in EPSG:4326 degrees
    const CRSBounds &bounds() const { return m_bounds; }

    /// Get the source dataset's sample size, in degrees
    double resolution() const { return m_resolution; }

    /// Get the tiling grid
    const GlobalGeodeticGrid &grid() const { return m_grid; }

    /// Get the warping options
    const TilerOptions &options() const { return m_options; }

    /// Get the deepest zoom level justified by the source dataset's resolution
    i_zoom maxZoomLevel() const { return m_grid.zoomForResolution(m_resolution); }

    /// Get the tiles spanned by the source dataset at a given zoom level
    TileBounds tileBoundsForZoom(i_zoom zoom) const {
        const TileCoordinate ll = m_grid.crsToTile(m_bounds.getLowerLeft(), zoom);
        const TileCoordinate ur = m_grid.crsToTile(m_bounds.getUpperRight(), zoom);
        return TileBounds(ll, ur);
    }

    /// Get the source dataset (borrowed)
    GDALDataset *dataset() const { return m_dataset; }

    /**
     * @brief Warp the raster covered by a tile into an in-memory dataset
     *
     * The result is a single-band `numSamples` x `numSamples` raster in EPSG:4326.
     *
     * Note: the warp produces exactly the requested number of samples. Reading
     * a full-size tile and decimating it afterwards (as RasterIO would) does not
     * keep the first and last samples on the tile's edges.
     *
     * @param coord The tile to extract
     * @param numSamples Samples along each side of the tile (at least 2)
     * @param[out] sampleExtent The extent the returned samples span, which is
     *             what the tile's lat/lon must be derived from. Note that under
     *             SamplingMode::HeightmapBorder this is *not* the tile's true
     *             extent.
     * @return The warped raster
     * @throws QmgcException if the warp fails
     */
    GdalDatasetPtr createRasterTile(const TileCoordinate &coord, int numSamples,
                                    CRSBounds &sampleExtent) const;

private:
    GDALDataset *m_dataset;      //!< The source raster (borrowed, not owned)
    GlobalGeodeticGrid m_grid;   //!< The tiling grid
    TilerOptions m_options;      //!< Warping options
    CRSBounds m_bounds;          //!< Source extent in EPSG:4326
    double m_resolution;         //!< Source sample size in degrees

    /// Determine the source extent and resolution in the grid's CRS
    void computeSourceGeometry();
};

} // namespace qmgc

#endif //EMODNET_QMGC_TILING_RASTER_TILER_H
