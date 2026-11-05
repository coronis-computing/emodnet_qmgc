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

#ifndef EMODNET_QMGC_LAYER_JSON_H
#define EMODNET_QMGC_LAYER_JSON_H

#include "tiling/geo_types.h"
#include <map>
#include <string>

/**
 * @brief Writes the layer.json file that Cesium requires at the root of a quantized-mesh tileset
 *
 * If the folder already contains a layer.json file (e.g., because the pyramid is being built in several runs of
 * qm_tiler, each one creating a different range of zooms), the zooms it lists and that are not in \p tilesPerZoom
 * are kept, and its bounds are merged with \p bounds.
 *
 * @param outDir The root folder of the tileset
 * @param tilesPerZoom The (inclusive) range of tiles created at each zoom
 * @param bounds The extent of the data, in EPSG:4326 degrees
 * @param withNormals Whether the tiles include the oct-encoded per-vertex normals extension
 * @param[out] error Description of the problem, when the function fails
 * @return true on success
 */
bool writeLayerJson(const std::string& outDir,
                    const std::map<int, qmgc::TileBounds>& tilesPerZoom,
                    const qmgc::CRSBounds& bounds,
                    bool withNormals,
                    std::string& error);

#endif //EMODNET_QMGC_LAYER_JSON_H
