#include "armor_finder/armor_finder.h"


void ArmorFinder::initUartParam() {

}

bool ArmorFinder::pipelineTargetPosition(cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right)
{
    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);


    return sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));

}



bool ArmorFinder::sendTargetByUart(float x, float y, float z) {
    // following lines are useful for aiming rotating target when the FPS is low
//    if(!targetPositionStreamControl(x, y)){
//        return false;
//    }

    armor_space_position_.x -= stereo_camera_param_.CAMERA_DISTANCE/2;
    //std::cout << armor_space_position_ << std::endl;
    armor_space_position_.z = 300;
    uart_.sendTarget(x, y, z);
    return true;
}



