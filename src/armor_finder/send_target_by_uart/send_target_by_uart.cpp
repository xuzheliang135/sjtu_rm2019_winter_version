
#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"


void ArmorFinder::initUartParam() {
    armor_type_ = NOT_FOUND;
}


void ArmorFinder::sendTargetByUart(float x, float y, float z) {
    uart_.sendTarget(x, y, z);
}



