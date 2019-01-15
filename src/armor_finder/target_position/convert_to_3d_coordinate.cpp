
#include "armor_finder/armor_finder.h"
#include "armor_finder/constant.h"

using namespace cv;
using std::cout;
using std::endl;

void ArmorFinder::initCameraParam() {
    stereo_camera_param_.CAMERA_DISTANCE = 21.2;  // cm
    stereo_camera_param_.FOCUS = 0.4 ;          // 4mm = 0.4cm
    //stereo_camera_param_.FOCUS = 666.7 ;          //
    stereo_camera_param_.LENGTH_PER_PIXAL = 0.247/640;  //0.3mm/640
    //stereo_camera_param_.LENGTH_PER_PIXAL = 1;

    stereo_camera_param_.HEIGHT_DIFF = -23;
}

bool ArmorFinder::convertToStereoscopicCoordinate(
        cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right, cv::Point3d &space_position) {

    double disparity; //视差
//    cout<<armor_box_left.tl()<<armor_box_right.tl()<<endl;
    disparity = abs(armor_box_left.x - armor_box_right.x) * stereo_camera_param_.LENGTH_PER_PIXAL;
    space_position.z = stereo_camera_param_.FOCUS * stereo_camera_param_.CAMERA_DISTANCE / disparity;
    space_position.x = space_position.z * (armor_box_left.x - SRC_WIDTH/2.) *
            stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS;
    space_position.y = space_position.z * (armor_box_left.y - SRC_HEIGHT/2.) *
            stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS;
    double x_right = space_position.z * (armor_box_right.x - SRC_WIDTH/2.) *
                                        stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS;
    double y_right = space_position.z * (armor_box_right.y - SRC_HEIGHT/2.) *
                     stereo_camera_param_.LENGTH_PER_PIXAL / stereo_camera_param_.FOCUS;
    //cout<<x_right<<" "<<y_right<<endl;
    return true;
}