//
// Created by Ricard Campos
//

#include "gzip_file_reader.h"

GZipFileReader::GZipFileReader( const std::string &filePath )
{
    m_file = gzopen(filePath.c_str(), "rb") ;
    m_pos = 0 ;
}



Byte GZipFileReader::readByte()
{
    return this->read<Byte>() ;
}



//! Skips a given number of bytes
void GZipFileReader::skipBytes( int numBytes ) {
    for ( int i = 0; i < numBytes; i++ ) {
        this->readByte() ;
    }
}



double GZipFileReader::readDouble()
{
    return this->read<double>() ;
}



float GZipFileReader::readFloat()
{
    return this->read<float>() ;
}



int GZipFileReader::readInt()
{
    return this->read<int>() ;
}



unsigned int GZipFileReader::readUInt()
{
    return this->read<unsigned int>() ;
}



short GZipFileReader::readShort()
{
    return this->read<short>() ;
}



unsigned short GZipFileReader::readUShort()
{
    return this->read<unsigned short>() ;
}