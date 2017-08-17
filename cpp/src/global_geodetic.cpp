//
// Created by Ricard Campos
//

#include "global_geodetic.h"

GlobalGeodetic::GlobalGeodetic( bool tmsCompatible, int tileSize = 256 )
{
    m_tileSize = tileSize ;
    if ( tmscompatible ) {
        m_resFact = 180.0 / self.tileSize ;
        m_numberOfLevelZeroTilesX = 2 ;
        m_numberOfLevelZeroTilesY = 1 ;
    }
    else {
        m_resFact = 360.0 / self.tileSize ;
        m_numberOfLevelZeroTilesX = 1 ;
        m_numberOfLevelZeroTilesY = 1 ;
    }
}