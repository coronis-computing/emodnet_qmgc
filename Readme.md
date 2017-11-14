# EMODnet Quantized Mesh generator for Cesium

Library for generating 

## Requirements

## Issues to take into account when implementing the quantized-mesh format

* The vertices and indices need to be optimized for both cache and fetching to be suitable for highwatermark encoding. We use a slightly modified version of the meshoptimizer library (https://github.com/zeux/meshoptimizer) to perform these steps.
