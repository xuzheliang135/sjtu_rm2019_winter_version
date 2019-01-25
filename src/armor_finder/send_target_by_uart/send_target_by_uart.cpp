
#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"


void ArmorFinder::initUartParam() {
    armor_type_ = NOT_FOUND;
}


bool ArmorFinder::sendTargetByUart(float x, float y, float z) {
//    if(!targetPositionStreamControl(x, y)){
//        return false;
//    }

    uart_.sendTarget(x, y, z);
    return true;
}



