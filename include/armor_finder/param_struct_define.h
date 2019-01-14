//
// Created by zhikun on 18-12-2.
//

#ifndef STEREOVISION_PARAM_DEFINE_H
#define STEREOVISION_PARAM_DEFINE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct LightBlob {
    cv::RotatedRect rect;
    std::vector<cv::Point> contour;
    explicit LightBlob(std::vector<cv::Point> &c) : contour(c) {
        rect = minAreaRect(c);
    };
    bool operator<(LightBlob &l2) { return this->rect.center.x < l2.rect.center.x; }
    bool operator<=(LightBlob &l2) { return this->rect.center.x <= l2.rect.center.x; }
    bool operator>(LightBlob &l2) { return this->rect.center.x > l2.rect.center.x; }
    bool operator>=(LightBlob &l2) { return this->rect.center.x >= l2.rect.center.x; }
};

struct LightBlobParam {
    int GRAY_THRESH;
    long CONTOUR_AREA_MAX;
    long CONTOUR_AREA_MIN;
    long CONTOUR_LENGTH_MIN;
    float CONTOUR_HW_RATIO_MAX;
    float CONTOUR_HW_RATIO_MIN;
    float CONTOUR_ANGLE_MAX;
};

struct LightCoupleParam{
    float TWIN_ANGEL_MAX;
    float TWIN_LENGTH_RATIO_MAX;
    float TWIN_DISTANCE_N_MIN;
    float TWIN_DISTANCE_N_MAX;
    float TWIN_DISTANCE_T_MAX;
    float TWIN_AREA_MAX;
    float TWIN_CENTER_POSITION_DIFF_RATIO;
};

struct StereoCameraPara {
    double CAMERA_DISTANCE;
    double FOCUS;
    double LENGTH_PER_PIXAL;
    int WIDTH, HEIGHT;
    int HEIGHT_DIFF;
};

struct ArmorSeekingParam{
    int BORDER_IGNORE;
    int BOX_EXTRA;

};

struct ArmorPridictParam{
    int ARMOR_POSITION_HISTORY_MAX_LENGTH;
};

enum ArmorType {
    NOT_FOUND,
    SMALL_ARMOR,
    LARGE_ARMOR,
    ARMOR_TYPE_SIZE
};

enum StateMachine{
    STAND_BY,
    SEARCHING_TARGET,
    TRACKING_TARGET
};

struct StateMachineParam{
    int THRESHOLD_FOUND_SEARCHING_TO_TRACKING;
    int THRESHOLD_UNFOUND_TRACKING_TO_SEARCHING;
};




#endif //STEREOVISION_PARAM_DEFINE_H
