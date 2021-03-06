# Developer Notes

## Decisions taken

* While the QuantizedMesh class has read/write capabilities for the quantized mesh format with both oct-encoded per-vertex normals and water masks, the quantized mesh tiles generated by qm_tiler do not include the water mask. 
* While simplification results in a higher zoom level could be used as starting point for lower levels in the pyramid, we choose to always sample for each tile the dataset at the resolution required by the zoom level. This is because we treat each tile individually, and we are not sure we can assure the simplification quality at the tiles' edges to be the as required.

## TODOs

* Decouple the tile limits from the tile simplifier. As it is now, the conversion ECEF coordinates needed by the point set simplification methods assumes that the input uses a lat/lon/height system. This is too restrictive, and should be generalized.
* Find a way to sample large rasters without getting out of memory.
* In dem2tin, it is not straightforward to use the point set methods with gdal rasters. In qm_tiler, since we use libctb for gathering the raster tiles, the tiles are already in the global geodetic SRS, so the transformations that we compute internally to ECEF coordinates assume that everything is in that SRS. However, because of how we have it implemented now, we don't have a straightforward way of computing this from a GDAL raster with an arbitrary SRS.

## Issues to take into account when implementing the quantized-mesh format
* The vertices and indices need to be optimized for both cache and fetching to be suitable for highwatermark encoding. We use a slightly modified version of the meshoptimizer library (https://github.com/zeux/meshoptimizer) to perform these steps.
