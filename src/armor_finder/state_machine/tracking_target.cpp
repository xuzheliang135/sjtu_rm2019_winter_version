#include "armor_finder/armor_finder.h"

using namespace cv;

bool ArmorFinder::stateTrackingTarget(cv::Mat &src_left, cv::Mat &src_right) {

    std::cout<<"dive into tracking"<<std::endl;
    /********************** tracking ***********************************************/
    track(kcf_tracker_left_, src_left, armor_box_left_);
    track(kcf_tracker_right_, src_right, armor_box_right_);
    showArmorBox("tracking boxes", src_left, armor_box_left_, src_right, armor_box_right_);

    /********************** convert to 3d coordinate *********************************/
    //convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);


    /********************** convert 3d coordinate back to two camera vision ***************/
    //showSpacePositionBackToStereoVision(src_left, src_right, armor_space_position_);


    /******************** predict the armor moving path *******************************/
    //predictArmorPosition(armor_space_position_, armor_predicted_position_);



    /*********************** send position by uart **************************************/
    //cout<<armor_space_position_<<endl;
    armor_space_position_.x += 5;
    armor_space_position_.y = armor_space_position_.y * 1.63 + 7.7;
    armor_space_position_.z = armor_space_position_.z * 1.24 - 42.4;


    sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));

    //cout<<armor_space_position_<<endl;
    return true;
}