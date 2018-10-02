//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#ifndef EMODNET_TOOLS_GZIP_FILE_WRITER_H
#define EMODNET_TOOLS_GZIP_FILE_WRITER_H

#include <zlib.h>
#include <string>
#include <cstring>

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


#endif //EMODNET_TOOLS_GZIP_FILE_WRITER_H
