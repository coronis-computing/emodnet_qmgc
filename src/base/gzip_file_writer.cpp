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
