//
// Author: Ricard Campos (ricardcd@gmail.com).
//

#ifndef EMODNET_TOOLS_MISC_UTILS_H
#define EMODNET_TOOLS_MISC_UTILS_H

static double remap(const double& value, const double& minOr, const double& maxOr, const double& minDest, const double& maxDest) {
    if ( maxOr-minOr == 0 ) // Avoid division by zero
        return 0 ;
    else
        return ( (value-minOr)/(maxOr-minOr) ) * (maxDest-minDest) + minDest ;
}

#endif //EMODNET_TOOLS_MISC_UTILS_H
