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

#ifndef EMODNET_QMGC_TILING_QMGC_EXCEPTION_H
#define EMODNET_QMGC_TILING_QMGC_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace qmgc {

/**
 * @class QmgcException
 * @brief A runtime error raised by the tiling machinery.
 *
 * Replaces cesium-terrain-builder's ctb::CTBException.
 */
class QmgcException : public std::runtime_error {
public:
    explicit QmgcException(const std::string &message) : std::runtime_error(message) {}
    explicit QmgcException(const char *message) : std::runtime_error(message) {}
};

} // namespace qmgc

#endif //EMODNET_QMGC_TILING_QMGC_EXCEPTION_H
