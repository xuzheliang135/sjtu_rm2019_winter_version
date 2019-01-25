#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::initTrackingParam(){
    track_param_.THRESHOLD_FOR_COUNT_NON_ZERO = 200;
    track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO = 0.5;

}


bool ArmorFinder::stateTrackingTarget(cv::Mat &src_left, cv::Mat &src_right) {

    /********************** tracking ***********************************************/
    track(kcf_tracker_left_, src_left, armor_box_left_);
    track(kcf_tracker_right_, src_right, armor_box_right_);
    if( (Rect2d(0, 0, 640, 480)&armor_box_left_).area() < armor_box_left_.area() ||
        (Rect2d(0, 0, 640, 480)&armor_box_right_).area() < armor_box_right_.area()) // avoid box touching edges
    {
        return false;
    }

    Mat roi_left = src_left.clone()(armor_box_left_);
    Mat roi_right = src_right.clone()(armor_box_right_);
    threshold(roi_left, roi_left, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
    threshold(roi_right, roi_right, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);

    if(abs(countNonZero(roi_left)-total_contour_area_left_) >= track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO * total_contour_area_left_ ||
        abs(countNonZero(roi_right)-total_contour_area_right_) >= track_param_.TRANSFER_RATIO_OF_TRACKING_AREA_NONZERO * total_contour_area_right_){
            return false;
    }

    //showArmorBox("tracking boxes", src_left, armor_box_left_, src_right, armor_box_right_);

    /********************** convert to 3d coordinate *********************************/
    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);

    /********************** convert 3d coordinate back to two camera vision ***************/
    //showSpacePositionBackToStereoVision(src_left, src_right, armor_space_position_);

    /******************** predict the armor moving path *******************************/
//    predictArmorPosition(armor_space_position_, armor_predicted_position_);


    /*********************** send position by uart **************************************/


    armor_space_position_.x -= stereo_camera_param_.CAMERA_DISTANCE/2;
    //std::cout << armor_space_position_ << std::endl;
    armor_space_position_.z = 300;
    return sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));

    return true;
}