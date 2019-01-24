#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;




bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left_light, cv::Mat &src_right_light) {

    /************************** find light blobs **********************************************/
    imagePreprocess(src_left_light, src_right_light, src_left_, src_right_);  // bayer hacking, to split blue and red

    piplineForFindLightBlob(src_left_light, src_right_light, light_blobs_left_real_, light_blobs_right_real_);

    /*************************** match light blobs***********************************/

    matchLightBlobVector(light_blobs_left_real_, armor_boxes_left_);
    matchLightBlobVector(light_blobs_right_real_, armor_boxes_right_);
    showArmorBoxVector("armor boxes", src_left_light, armor_boxes_left_, src_right_light, armor_boxes_right_);
    for (auto &armor_box:armor_boxes_left_) {
        armor_num_left.emplace_back(recognize_digits(src_raw_left_(armor_box)));
    }
    for (auto &armor_box:armor_boxes_right_) {
        armor_num_right.emplace_back(recognize_digits(src_raw_right_(armor_box)));
    }

    bool state_match = matchTwoArmorBox(armor_boxes_left_, armor_boxes_right_, armor_box_left_, armor_box_right_);
    if(!state_match) {return false;}


    /********************** convert to 3d coordinate *********************************/
    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);

    /********************** convert 3d coordinate back to two camera vision ***************/
    //showSpacePositionBackToStereoVision(src_left, src_right, armor_space_position_);

    /******************** predict the armor moving path *******************************/
//    predictArmorPosition(armor_space_position_, armor_predicted_position_);


    /*********************** send position by uart **************************************/


    armor_space_position_.x -= stereo_camera_param_.CAMERA_DISTANCE/2;
    cout << armor_space_position_ << endl;

    sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));

    return true;

}