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
    armor_predict_param_.time_predict = 500;   // 500 ms
}

void ArmorFinder::manageHistorySpacePosition(const cv::Point3d &space_position) {
    clock_t cur_time = clock();
    armor_history_positions_.push_back(space_position);
    time_serial.push_back(cur_time);
    if(armor_history_positions_.size() > armor_predict_param_.ARMOR_POSITION_HISTORY_MAX_LENGTH)
    {
        armor_history_positions_.erase(armor_history_positions_.begin());
        time_serial.erase(time_serial.begin());
    }
}


bool ArmorFinder::predictArmorPosition(cv::Point3d &armor_position,
                                        cv::Point3d &armor_predicted_position)
{
    manageHistorySpacePosition(armor_position);

    vector<clock_t> data_time_serial;
    for(int i = 0; i < time_serial.size(); i++)
    {
        data_time_serial.push_back(time_serial[i] - time_serial.back());
    }
    curve_fitting_.predictPositionOverTime(armor_history_positions_, data_time_serial,
            armor_predict_param_.time_predict, armor_predicted_position);

    return true;
}
