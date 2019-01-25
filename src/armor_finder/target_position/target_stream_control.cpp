#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"

#include <iostream>




bool ArmorFinder::targetPositionStreamControl(float x, float y) {

    static float last_x = 0;
    double cur_diff = abs(x - last_x);
    bool willSikp = cur_diff <= position_diff*1.5;

    double alpha = 0.8;
    position_diff =  alpha * cur_diff + (1-alpha) * position_diff ;
    last_x = x;

    return willSikp;
}

