#include "armor_finder/armor_finder.h"

using namespace cv;

bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left, cv::Mat &src_right) {

    sendTargetByUart(0, 0, 0);        // used to output fps

    static Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(1, 4));
    erode(src_left, src_left, kernel_erode);
    erode(src_right, src_right, kernel_erode);
    //showTwoImages("erode", src_left, src_right);

    static Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(2, 4));
    dilate(src_left, src_left, kernel_dilate);
    dilate(src_right, src_right, kernel_dilate);
    //showTwoImages("dilate", src_left, src_right);

    static Mat kernel_erode2 = getStructuringElement(MORPH_RECT, Size(2, 4));
    erode(src_left, src_left, kernel_erode2);
    erode(src_right, src_right, kernel_erode2);
    //showTwoImages("erode2", src_left, src_right);

    static Mat kernel_dilate2 = getStructuringElement(MORPH_RECT, Size(3, 6));
    dilate(src_left, src_left, kernel_dilate2);
    dilate(src_right, src_right, kernel_dilate2);
    //showTwoImages("dilate2", src_left, src_right);

    float alpha = 1.5;
    int beta = 0;
    src_left.convertTo(src_left, -1, alpha, beta);
    src_right.convertTo(src_right, -1, alpha, beta);
    showTwoImages("enlighted", src_left, src_right);


    /************* debug code ***************************************/
    threshold(src_left, src_bin_left_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
    threshold(src_right, src_bin_right_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
    showTwoImages("bin", src_bin_left_, src_bin_right_);
    /*********************************************************/

    /************************** find light blobs **********************************************/
    light_blobs_left_.clear(); light_blobs_right_.clear();
    bool state_left, state_right;
    state_left = findLightBlob(src_left, light_blobs_left_);
    state_right = findLightBlob(src_right, light_blobs_right_);
    if(!(state_left && state_right)) {return false;}
    showContours("light contours", src_left, light_blobs_left_, src_right, light_blobs_right_);



    /*************************** match light blobs***********************************/
    state_left = matchLightBlob(light_blobs_left_, armor_box_left_);
    state_right = matchLightBlob(light_blobs_right_, armor_box_right_);
    if(!(state_left && state_right)) {return false;}
    showArmorBox("armor boxes", src_left, armor_box_left_, src_right, armor_box_right_);

    return false;


    /********************** convert to 3d coordinate *********************************/
    //convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);


    /********************** convert 3d coordinate back to two camera vision ***************/
    //showSpacePositionBackToStereoVision(src_left, src_right, armor_space_position_);


    /******************** predict the armor moving path *******************************/
    //predictArmorPosition(armor_space_position_, armor_predicted_position_);



    /*********************** send position by uart **************************************/
//    //cout<<armor_space_position_<<endl;
//    armor_space_position_.x += 5;
//    armor_space_position_.y = armor_space_position_.y * 1.63 + 7.7;
//    armor_space_position_.z = armor_space_position_.z * 1.24 - 42.4;

//    sendTargetByUart(
//            static_cast<float>(armor_space_position_.x),
//            static_cast<float>(armor_space_position_.y),
//            static_cast<float>(armor_space_position_.z));

    return true;

}