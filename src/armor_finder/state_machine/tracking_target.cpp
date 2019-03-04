#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::initTrackingParam(){
    track_param_.THRESHOLD_FOR_COUNT_NON_ZERO = 200;
    track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO = 0.5;

}


bool ArmorFinder::stateTrackingTarget(cv::Mat &src_left) {

    /********************** tracking ***********************************************/
    track(kcf_tracker_left_, src_left, armor_box_left_);
    if ((Rect2d(0, 0, 640, 480) & armor_box_left_).area() < armor_box_left_.area()) // avoid box touching edges
    {
        return false;
    }

    Mat roi_left = src_left.clone()(armor_box_left_);
    threshold(roi_left, roi_left, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);

    if (abs(countNonZero(roi_left) - total_contour_area_left_) >=
        track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO * total_contour_area_left_) {
            return false;
    }

    showArmorBox("tracking boxes", src_left, armor_box_left_);

    /********************** convert to 3d coordinate *********************************/
//    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);
//todo
    /*************** a predict function for moving target with only constant speed *******************/
    targetTrackPositionStreamControl(armor_space_position_);

    /********************** send it by uart and adjust the original point to the center *************/
    return sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));
}