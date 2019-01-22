#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <time.h>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"


void ArmorFinder::initArmorPredictParam(){
    armor_predict_param_.ARMOR_POSITION_HISTORY_MAX_LENGTH = 100;
}

void ArmorFinder::manageHistorySpacePosition(const cv::Point3d &space_position) {
    armor_history_positions_.push_back(space_position);
    if(armor_history_positions_.size() > armor_predict_param_.ARMOR_POSITION_HISTORY_MAX_LENGTH)
    {
        armor_history_positions_.erase(armor_history_positions_.begin());
    }
}


bool ArmorFinder::predictArmorPosition(cv::Point3d &armor_position,
                                        cv::Point3d &armor_predicted_position)
{
    manageHistorySpacePosition(armor_position);

    return true;
}
