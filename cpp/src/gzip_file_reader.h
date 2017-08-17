//
// Created by ricard on 17/08/17.
//

#ifndef EMODNET_TOOLS_GZIP_FILE_READER_H
#define EMODNET_TOOLS_GZIP_FILE_READER_H

#include <zlib.h>
#include <string>
#include <cstring>

class GZipFileReader {

public:
    // Typedefs
    typedef unsigned char Byte ;

    //! Constructor
    GZipFileReader( const std::string &filePath ) ;

    //! Check if the file is open
    bool isFileOpen() { return m_file != NULL ; }

    //! Reads a byte
    Byte readByte() ;

    //! Skips a given number of bytes
    void skipBytes( int numBytes ) ;

    //! Reads a double
    double readDouble() ;

    //! Reads a float
    float readFloat() ;

    //! Reads a int
    int readInt() ;

    //! Reads an unsigned int
    unsigned int readUInt() ;

    //! Reads a short
    short readShort() ;

    //! Reads an unsigned short
    unsigned short readUShort() ;

    //! Generic templated read function
    template <typename T>
    T read() {
        // Size of the type used
        int sz = sizeof(T) ;

        // Read from the GZip file
        Byte buffer[sz];
        int bytesRead = gzread( m_file, buffer, sz ) ;
        T ret ;
        memcpy( &ret, &buffer[0], sz ) ;

        // Update the position of the pointer in the file
        m_pos += sz ;

        return ret ;
    }

    //! Returns the position on the file (i.e., read byte counter)
    int getPos() { return m_pos ; }

    //! Closes the file
    bool close() { return gzclose(m_file) ; }

private:
    gzFile m_file ; //! The file to read
    int m_pos ;     //! The position on the file (byte index)
};


#endif //EMODNET_TOOLS_GZIP_FILE_READER_H
