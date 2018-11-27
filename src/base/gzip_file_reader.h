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

#ifndef EMODNET_QMGC_GZIP_FILE_READER_H
#define EMODNET_QMGC_GZIP_FILE_READER_H

#include <zlib.h>
#include <string>
#include <cstring>

/**
 * @class GZipFileReader
 * @brief Helper class to read a GZip file.
 */
class GZipFileReader {

public:
    // Typedefs
    typedef unsigned char Byte;

    /// Constructor
    GZipFileReader( const std::string &filePath );

    /// Check if the file is open
    bool isFileOpen() { return m_file != NULL; }

    /// Reads a byte
    Byte readByte();

    /// Skips a given number of bytes
    void skipBytes(int numBytes);

    /// Reads a double
    double readDouble();

    /// Reads a float
    float readFloat();

    /// Reads a int
    int readInt();

    /// Reads an unsigned int
    unsigned int readUInt();

    /// Reads a short
    short readShort();

    /// Reads an unsigned short
    unsigned short readUShort();

    /// Reads a char
    char readChar();

    /// Reads an unsigned char
    unsigned char readUChar();

    /// Generic templated read function
    template <typename T>
    T read() {
        // Size of the type used
        int sz = sizeof(T);

        // Read from the GZip file
        Byte buffer[sz];
        int bytesRead = gzread(m_file, buffer, sz);
        T ret;
        memcpy(&ret, &buffer[0], sz);

        // Update the position of the pointer in the file
        m_pos += sz;

        return ret;
    }

    /// Returns the position on the file (i.e., read byte counter)
    int getPos() { return m_pos; }

    /// Closes the file
    bool close() { return gzclose(m_file); }

    /// Indicates the end of file
    bool eof() { return gzeof(m_file); }

private:
    gzFile m_file; //!< The file to read
    int m_pos;     //!< The position on the file (byte index)
};


#endif //EMODNET_QMGC_GZIP_FILE_READER_H
