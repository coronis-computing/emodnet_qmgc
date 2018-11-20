//
// Created by Ricard Campos on 11/15/18.
//

#include "tin_creator.h"
#include "crs_conversions.h"

namespace TinCreation {

//std::vector<Point_3>
//TinCreationStrategy::
//convertUVHToECEF(const std::vector<Point_3> &pts) const {
//    if (this->getMaxX() < 0) {
//        std::cout << "Doing nothing" << std::endl ;
//        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
//        return pts;
//    }
//
//    // Points in ECEF coordinates
//    std::vector<Point_3> ecefPoints;
//    ecefPoints.reserve(pts.size());
//    for (std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
//        // From UVH to lat/lon/height
//        double lat = this->getMinY() + ((this->getMaxY() - this->getMinY()) * it->y());
//        double lon = this->getMinX() + ((this->getMaxX() - this->getMinX()) * it->x());
//        double h = this->getMinZ() + ((this->getMaxZ() - this->getMinZ()) * it->z());
//
//        double tmpx, tmpy, tmpz;
//        crs_conversions::llh2ecef(lat, lon, h,
//                                  tmpx, tmpy, tmpz);
//
//        ecefPoints.emplace_back(Point_3(tmpx, tmpy, tmpz));
//    }
//
//    return ecefPoints;
//}
//
//
//std::vector<Point_3>
//TinCreationStrategy::
//convertECEFToUVH(const std::vector<Point_3> &pts) const {
//    if (this->getMaxX() < 0) {
//        // The limits were not set, probably because the input to decimate is a mesh, and the scale is supposed to be metric already
//        return pts;
//    }
//
//    // Points in ECEF coordinates
//    std::vector<Point_3> uvhPoints;
//    uvhPoints.reserve(pts.size());
//    for (std::vector<Point_3>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
////        std::cout << "x = " << (*it).x() << " / y = " << (*it).y() << " / z = " << (*it).z() << std::endl;
//
//        // From ECEF to lat/lon/height
//        double lat, lon, height;
//        crs_conversions::ecef2llh((*it).x(), (*it).y(), (*it).z(),
//                                  lat, lon, height);
//
////        if (lat > this->getMaxY())
////            lat = this->getMaxY();
////        else if (lat < this->getMinY())
////            lat = this->getMinY();
////        if (lon > this->getMaxX())
////            lon = lon + this->getMinX();
////        else if (lon < this->getMaxX())
////            lon = this->getMaxX();
//
////        std::cout << "this->getMinX() = " << this->getMinX() << " / this->getMaxX() = " << this->getMaxX() << std::endl;
////        std::cout << "this->getMinY() = " << this->getMinY() << " / this->getMaxY() = " << this->getMaxY() << std::endl;
////
////        std::cout << "lat = " << lat << " / lon = " << lon << " / h = " << height << std::endl;
//
//        // Scale to local U/V/H
//        double u = (lon - this->getMinX()) / (this->getMaxX() - this->getMinX());
//        double v = (lat - this->getMinY()) / (this->getMaxY() - this->getMinY());
//        double h = (this->getMaxZ() - this->getMinZ()) < std::numeric_limits<double>::epsilon()
//                   ? // Avoid division by 0 ((this->getMaxZ() - this->getMinZ()) == 0 in flat tiles!)
//                   height - this->getMinZ()
//                   : (height - this->getMinZ()) / (this->getMaxZ() - this->getMinZ());
//
////        std::cout << "u = " << u << " / v = " << v << " / h = " << h << std::endl;
//
//
////        if ( u > 1 ) u = 0;
////        if ( u < 0 ) u = 1;
////        if ( v > 1 ) v = 1;
////        if ( v < 0 ) v = 0;
//
////        std::cout << "u = " << u << " / v = " << v << " / h = " << h << std::endl;
//
//        uvhPoints.emplace_back(Point_3(u, v, h));
//    }
//
//    return uvhPoints;
//}

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