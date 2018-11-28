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

#include "tin_creator.h"
#include "crs_conversions.h"

namespace TinCreation {

std::vector<Point_3>
TinCreationStrategy::
convertUVHToECEF(const std::vector<Point_3> &pts) const {
    if (!this->hasOriginalBoundingBox()) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return pts;
    }

    // Points in ECEF coordinates
    std::vector<Point_3> ecefPoints;
    ecefPoints.reserve(pts.size());
    for (std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
        ecefPoints.emplace_back(convertUVHToECEF(*it));
    }

    return ecefPoints;
}


std::vector<Point_3>
TinCreationStrategy::
convertECEFToUVH(const std::vector<Point_3> &pts) const {
    if (!this->hasOriginalBoundingBox()) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return pts;
    }

    // Points in ECEF coordinates
    std::vector<Point_3> uvhPoints;
    uvhPoints.reserve(pts.size());
    for (std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
        uvhPoints.emplace_back(convertECEFToUVH(*it));
    }

    return uvhPoints;
}

Point_3 TinCreationStrategy::convertUVHToECEF(const Point_3& p) const
{
    if (!this->hasOriginalBoundingBox()) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return p;
    }

    // From UVH to lat/lon/height
    double lat = this->getMinY() + ((this->getMaxY() - this->getMinY()) * p.y());
    double lon = this->getMinX() + ((this->getMaxX() - this->getMinX()) * p.x());
    double h = this->getMinZ() + ((this->getMaxZ() - this->getMinZ()) * p.z());

    double tmpx, tmpy, tmpz;
    crs_conversions::llh2ecef(lat, lon, h,
                              tmpx, tmpy, tmpz);

    return Point_3(tmpx, tmpy, tmpz);
}

/// Convert points from local UVH to ECEF given the limits of the tile
// This conversion is used by some point set simplification methods requiring metric coordinates
Point_3 TinCreationStrategy::convertECEFToUVH(const Point_3& p) const
{
    if (!this->hasOriginalBoundingBox()) {
        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
        return p;
    }

    // From ECEF to lat/lon/height
    double lat, lon, height;
    crs_conversions::ecef2llh(p.x(), p.y(), p.z(),
                              lat, lon, height);

    // Scale to local U/V/H
    double u = (lon - this->getMinX()) / (this->getMaxX() - this->getMinX());
    double v = (lat - this->getMinY()) / (this->getMaxY() - this->getMinY());
    double h = (this->getMaxZ() - this->getMinZ()) < std::numeric_limits<double>::epsilon()
               ? // Avoid division by 0 ((this->getMaxZ() - this->getMinZ()) == 0 in flat tiles!)
               height - this->getMinZ()
               : (height - this->getMinZ()) / (this->getMaxZ() - this->getMinZ());

    return Point_3(u, v, h);
}

} // End namespace TinCreation