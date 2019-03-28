#include "armor_finder/armor_finder.h"


void ArmorFinder::initUartParam() {

}


bool ArmorFinder::sendTargetByUart(float x, float y, float z) {

    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;
    uart_.sendTarget(x, y - 2, z);
    return true;
}



