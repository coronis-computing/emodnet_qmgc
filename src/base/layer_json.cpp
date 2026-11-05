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

#include "layer_json.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <fstream>

using json = nlohmann::json;

bool writeLayerJson(const std::string& outDir,
                    const std::map<int, qmgc::TileBounds>& tilesPerZoom,
                    const qmgc::CRSBounds& bounds,
                    bool withNormals,
                    std::string& error)
{
    const std::string filePath = outDir + "/layer.json";

    // The tiles of each zoom, starting with the ones listed in an already existing layer.json
    std::map<int, json> available;
    double west = std::max(bounds.getMinX(), -180.0), south = std::max(bounds.getMinY(), -90.0);
    double east = std::min(bounds.getMaxX(), 180.0), north = std::min(bounds.getMaxY(), 90.0);
    std::ifstream ifs(filePath);
    if (ifs) {
        try {
            const json previous = json::parse(ifs);
            const json& prevAvailable = previous.at("available");
            for (std::size_t zoom = 0; zoom < prevAvailable.size(); zoom++) {
                if (!prevAvailable[zoom].empty())
                    available[static_cast<int>(zoom)] = prevAvailable[zoom];
            }
            const json& prevBounds = previous.at("bounds");
            west = std::min(west, prevBounds.at(0).get<double>());
            south = std::min(south, prevBounds.at(1).get<double>());
            east = std::max(east, prevBounds.at(2).get<double>());
            north = std::max(north, prevBounds.at(3).get<double>());
        }
        catch (const json::exception& e) {
            error = "Could not parse the existing " + filePath + " (" + e.what() + "). Delete it to create a new one";
            return false;
        }
    }
    for (const auto& zt : tilesPerZoom) {
        const qmgc::TileBounds& tb = zt.second;
        available[zt.first] = json::array({ {{"startX", tb.getMinX()}, {"startY", tb.getMinY()},
                                             {"endX", tb.getMaxX()}, {"endY", tb.getMaxY()}} });
    }
    if (available.empty()) {
        error = "No tiles to list in " + filePath;
        return false;
    }

    // Note: Cesium indexes "available" by zoom, so it must start at zoom 0 even if the pyramid does not
    const int minZoom = available.begin()->first;
    const int maxZoom = available.rbegin()->first;
    json availableArray = json::array();
    for (int zoom = 0; zoom <= maxZoom; zoom++) {
        auto it = available.find(zoom);
        availableArray.push_back(it != available.end() ? it->second : json::array());
    }

    json layer = {
        {"tilejson", "2.1.0"},
        {"format", "quantized-mesh-1.0"},
        {"version", "1.2.0"},
        {"scheme", "tms"},
        {"tiles", {"{z}/{x}/{y}.terrain?v={version}"}},
        {"attribution", "Created using the EMODnet Quantized Mesh Generator for Cesium"},
        {"projection", "EPSG:4326"},
        {"minzoom", minZoom},
        {"maxzoom", maxZoom},
        {"bounds", {west, south, east, north}},
        {"available", availableArray}
    };
    if (withNormals) {
        // Cesium only requests and decodes the normals if the extension is listed here
        layer["extensions"] = {"octvertexnormals"};
    }

    std::ofstream ofs(filePath);
    if (!ofs) {
        error = "Could not open " + filePath + " for writing";
        return false;
    }
    ofs << layer.dump(4) << std::endl;
    if (!ofs) {
        error = "Could not write " + filePath;
        return false;
    }
    return true;
}
