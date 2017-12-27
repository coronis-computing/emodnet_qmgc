//
// Created by Ricard Campos (rcampos@eia.udg.edu)
//

#include "gzip_file_writer.h"


GZipFileWriter::GZipFileWriter( const std::string &filePath )
{
    m_file = gzopen(filePath.c_str(), "wb") ;
    m_pos = 0 ;
}



int GZipFileWriter::writeByte( const Byte &b )
{
    return this->write<Byte>(b) ;
}



int GZipFileWriter::writeDouble( const double &d )
{
    return this->write<double>(d) ;
}



int GZipFileWriter::writeFloat( const float &f )
{
    return this->write<float>(f) ;
}



int GZipFileWriter::writeInt( const int &i )
{
    return this->write<int>(i) ;
}



int GZipFileWriter::writeUInt( const unsigned int &u )
{
    return this->write<unsigned int>(u) ;
}



int GZipFileWriter::writeShort( const short &s )
{
    return this->write<short>(s) ;
}



int GZipFileWriter::writeUShort( const unsigned short &u )
{
    return this->write<unsigned short>(u) ;
}



int GZipFileWriter::writeChar( const char &c )
{
    return this->write<char>(c) ;
}



int GZipFileWriter::writeUChar( const unsigned char &c )
{
    return this->write<unsigned char>(c) ;
}
