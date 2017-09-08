//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_QUANTIZED_MESH_ITERATOR_H
#define EMODNET_TOOLS_QUANTIZED_MESH_ITERATOR_H


/**
 * @file TerrainIterator.hpp
 * @brief This declares the `TerrainIterator` class
 */

#include "quantized_mesh_tile.h"
#include "quantized_mesh_tiler.h"
#include <ctb.hpp>

/**
 * @brief This forward iterates over all `QuantizedMeshTile`s in a `QuantizedMeshTiler`
 *
 * Instances of this class take a `QuantizedMeshTiler` in the constructor and are used
 * to forward iterate over all tiles in the tiler, returning a `QuantizedMeshTile *`
 * when dereferenced.  It is the caller's responsibility to call `delete` on the
 * tile.
 */
class QuantizedMeshIterator : public ctb::TilerIterator
{
public:

    /// Instantiate an iterator with a tiler
    QuantizedMeshIterator(const QuantizedMeshTiler &tiler):
            QuantizedMeshIterator(tiler, tiler.maxZoomLevel(), 0)
    {}

    /// The target constructor
    QuantizedMeshIterator(const QuantizedMeshTiler &tiler, ctb::i_zoom startZoom, ctb::i_zoom endZoom):
            TilerIterator(tiler, startZoom, endZoom)
    {}

    virtual QuantizedMeshTile*
    operator*() const override {
        return static_cast<QuantizedMeshTile *>(ctb::TilerIterator::operator*());
    }
};

#endif //EMODNET_TOOLS_QUANTIZED_MESH_ITERATOR_H
