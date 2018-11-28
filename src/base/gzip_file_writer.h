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

#ifndef EMODNET_QMGC_GZIP_FILE_WRITER_H
#define EMODNET_QMGC_GZIP_FILE_WRITER_H

#include <zlib.h>
#include <string>
#include <cstring>

/**
 * @class GZipFileWriter
 * @brief Helper class to write a GZip file.
 */
class GZipFileWriter {
public:
    // Typedefs
    typedef unsigned char Byte ;

    /// Constructor
    GZipFileWriter( const std::string &filePath ) ;

    /// Check if the file is open
    bool isFileOpen() { return m_file != NULL ; }

    /// Writes a byte
    int writeByte( const Byte &b ) ;

    /// Writes a double
    int writeDouble( const double &d ) ;

    /// Writes a float
    int writeFloat( const float &f ) ;

    /// Writes an int
    int writeInt( const int &i ) ;

    /// Writes an unsigned int
    int writeUInt( const unsigned int &u ) ;

    /// Writes a short
    int writeShort( const short &s ) ;

    /// Writes an unsigned short
    int writeUShort( const unsigned short &u ) ;

    /// Writes a char
    int writeChar( const char &c ) ;

    /// Writes an unsigned char
    int writeUChar( const unsigned char &c ) ;

    /**
     * @brief Generic templated write function
     *
     * @return The number of written bytes, 0 means error
     */
    template <typename T>
    int write( T val ) {
        // Size of the type used
        int sz = sizeof(T) ;

        // Write to the GZip file
//        Byte buffer[sz];
//        memcpy( &buffer[0], &val, sz ) ;
        int bytesWritten = gzwrite( m_file, reinterpret_cast<char *>(&val), sz ) ;

        // Update the position of the pointer in the file
        m_pos += sz ;

        return bytesWritten ;
    }

    /// Returns the position on the file (i.e., read byte counter)
    int getPos() { return m_pos ; }

    /// Closes the file
    bool close() { return gzclose(m_file) ; }

private:
    gzFile m_file ; //!< The file to read
    int m_pos ;     //!< The position on the file (byte index)
};


#endif //EMODNET_QMGC_GZIP_FILE_WRITER_H
