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



char GZipFileReader::readChar()
{
    return this->read<char>() ;
}



unsigned char GZipFileReader::readUChar()
{
    return this->read<unsigned char>() ;
}