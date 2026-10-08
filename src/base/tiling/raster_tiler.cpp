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

#include "raster_tiler.h"
#include <gdal_alg.h>
#include <gdal_utils.h>
#include <ogr_spatialref.h>
#include <cpl_conv.h>
#include <cpl_string.h>
#include <cmath>
#include <cstdio>
#include <string>

namespace qmgc {

namespace {

/// Append a number to an argument list with enough digits to round trip exactly
void addNumber(CPLStringList &args, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    args.AddString(buffer);
}

/// Append a number to an argument list
void addNumber(CPLStringList &args, int value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%d", value);
    args.AddString(buffer);
}

/// Describe a tile as "z/x/y", for error messages
std::string describe(const TileCoordinate &coord) {
    return std::to_string(coord.zoom) + "/" + std::to_string(coord.x) + "/" +
           std::to_string(coord.y);
}

} // namespace

RasterTiler::RasterTiler(GDALDataset *dataset,
                         const GlobalGeodeticGrid &grid,
                         const TilerOptions &options)
        : m_dataset(dataset)
        , m_grid(grid)
        , m_options(options)
        , m_bounds()
        , m_resolution(0.0) {
    computeSourceGeometry();
}

void RasterTiler::computeSourceGeometry() {
    if (m_dataset == nullptr)
        throw QmgcException("No GDAL dataset was provided to the tiler");
    if (m_dataset->GetRasterCount() < 1)
        throw QmgcException("At least one band must be present in the GDAL dataset");

    if (m_dataset->GetSpatialRef() == nullptr)
        throw QmgcException("The source dataset does not have a spatial reference system assigned");

    // The target reference system, in traditional (longitude, latitude) axis order.
    //
    // Note: this call is what cesium-terrain-builder is missing. Since GDAL 3,
    // importFromEPSG(4326) yields the authority-compliant (latitude, longitude)
    // order, so without it every transformed coordinate comes out with its axes
    // swapped.
    OGRSpatialReference dstSRS;
    if (dstSRS.importFromEPSG(4326) != OGRERR_NONE)
        throw QmgcException("Could not create the EPSG:4326 spatial reference system");
    dstSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    char *dstWkt = nullptr;
    if (dstSRS.exportToWkt(&dstWkt) != OGRERR_NONE) {
        CPLFree(dstWkt);
        throw QmgcException("Could not export the EPSG:4326 spatial reference system to WKT");
    }

    char **transformOptions = CSLSetNameValue(nullptr, "DST_SRS", dstWkt);
    CPLFree(dstWkt);

    // The source reference system is taken from the dataset itself
    void *transformer = GDALCreateGenImgProjTransformer2(
            static_cast<GDALDatasetH>(m_dataset), nullptr, transformOptions);
    CSLDestroy(transformOptions);

    if (transformer == nullptr)
        throw QmgcException("Could not create a transformer from the source dataset to EPSG:4326: " +
                            std::string(CPLGetLastErrorMsg()));

    // Let GDAL work out the extent and the natural resolution of the reprojected
    // dataset. Unlike transforming the four corners by hand, this densifies the
    // edges, so it does not under-cover curved projections such as polar
    // stereographic or a wide Lambert conformal conic.
    double geoTransform[6];
    double extent[4];
    int pixels = 0;
    int lines = 0;
    const CPLErr err = GDALSuggestedWarpOutput2(static_cast<GDALDatasetH>(m_dataset),
                                                GDALGenImgProjTransform,
                                                transformer,
                                                geoTransform,
                                                &pixels,
                                                &lines,
                                                extent,
                                                0);
    GDALDestroyGenImgProjTransformer(transformer);

    if (err != CE_None)
        throw QmgcException("Could not determine the extent of the source dataset in EPSG:4326: " +
                            std::string(CPLGetLastErrorMsg()));

    m_bounds = CRSBounds(extent[0], extent[1], extent[2], extent[3]);
    m_resolution = std::abs(geoTransform[1]);

    if (m_resolution <= 0.0)
        throw QmgcException("The source dataset resolution in EPSG:4326 is not positive");
}

GdalDatasetPtr RasterTiler::createRasterTile(const TileCoordinate &coord,
                                             int numSamples,
                                             CRSBounds &sampleExtent) const {
    const TileWindow window = computeTileWindow(m_grid, coord, m_options.samplingMode, numSamples);
    sampleExtent = window.sampleExtent;

    CPLStringList args;

    // Warp into memory: the caller reads the whole thing immediately
    args.AddString("-of");
    args.AddString("MEM");

    args.AddString("-t_srs");
    args.AddString("EPSG:4326");

    // Only band 1 carries elevation
    args.AddString("-b");
    args.AddString("1");

    // The extent covered by the output, i.e. the outer edges of its samples
    args.AddString("-te");
    addNumber(args, window.warpExtent.getMinX());
    addNumber(args, window.warpExtent.getMinY());
    addNumber(args, window.warpExtent.getMaxX());
    addNumber(args, window.warpExtent.getMaxY());

    args.AddString("-ts");
    addNumber(args, numSamples);
    addNumber(args, numSamples);

    args.AddString("-r");
    args.AddString(m_options.resampleAlg.c_str());

    args.AddString("-et");
    addNumber(args, m_options.errorThreshold);

    // Overview selection. This is GDAL's own, maintained implementation of what
    // cesium-terrain-builder hand-rolled, and it is what keeps coarse zoom levels
    // of a very large source raster tractable.
    args.AddString("-ovr");
    args.AddString(m_options.useOverviews ? "AUTO" : "NONE");

    // Tiles are generated in parallel by the caller, so warping each one across
    // every core just oversubscribes the machine.
    args.AddString("-wo");
    args.AddString(("NUM_THREADS=" + std::to_string(m_options.warpThreads)).c_str());

    // Declaring nodata keeps the resampler from averaging it into valid samples
    if (m_options.excludeNoDataFromResampling) {
        int hasNoData = FALSE;
        const double noDataValue = m_dataset->GetRasterBand(1)->GetNoDataValue(&hasNoData);
        if (hasNoData) {
            args.AddString("-srcnodata");
            addNumber(args, noDataValue);
            args.AddString("-dstnodata");
            addNumber(args, noDataValue);
        }
    }

    GDALWarpAppOptions *warpOptions = GDALWarpAppOptionsNew(args.List(), nullptr);
    if (warpOptions == nullptr)
        throw QmgcException("Could not create the warp options for tile " + describe(coord) + ": " +
                            std::string(CPLGetLastErrorMsg()));

    GDALDatasetH srcDataset = static_cast<GDALDatasetH>(m_dataset);
    int usageError = FALSE;
    GDALDatasetH warped = GDALWarp("", nullptr, 1, &srcDataset, warpOptions, &usageError);
    GDALWarpAppOptionsFree(warpOptions);

    if (warped == nullptr)
        throw QmgcException("Could not warp the raster for tile " + describe(coord) + ": " +
                            std::string(CPLGetLastErrorMsg()));

    return GdalDatasetPtr(static_cast<GDALDataset *>(warped));
}

} // namespace qmgc
