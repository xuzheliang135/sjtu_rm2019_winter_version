#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;

cv::Rect normalize(cv::Rect roi) {
    roi.width *= 2;
    roi.height *= 2;
    if ((roi.x + roi.width) > 320)roi.width = 320 - roi.x;
    if (roi.y + roi.height > 240)roi.height = 240 - roi.y;
    if (roi.x < 0)roi.x = 0;
    if (roi.y < 0)roi.y = 0;
    if (roi.height <= 0)roi.height = 1;
    if (roi.width <= 0)roi.width = 1;
    return roi;
}

void ArmorFinder::replace_img(cv::Mat &src, cv::Mat &origin, vector<LightBlob> &light_blobs) {
    Mat tmp;
    resize(origin, tmp, Size(320, 240));
    for (auto &a:light_blobs) {
        Rect rect = normalize(a.rect.boundingRect());
        tmp(rect).copyTo(src(rect));
    }
}

void drawRotatedRectangle(Mat &img, const RotatedRect &rect) {
    Point2f points[4];
    rect.points(points);
    for (int i = 0; i < 3; i++)line(img, points[i], points[i + 1], Scalar(255, 0, 0));
    line(img, points[3], points[0], Scalar(255, 0, 0));
}

bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left, cv::Mat &src_right) {


    static Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(1, 4));
    erode(src_left, src_left, kernel_erode);
    erode(src_right, src_right, kernel_erode);
    //showTwoImages("erode", src_left, src_right);

    static Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(2, 4));
    dilate(src_left, src_left, kernel_dilate);
    dilate(src_right, src_right, kernel_dilate);
    //showTwoImages("dilate", src_left, src_right);
//
//    static Mat kernel_erode2 = getStructuringElement(MORPH_RECT, Size(2, 4));
//    erode(src_left, src_left, kernel_erode2);
//    erode(src_right, src_right, kernel_erode2);
//    //showTwoImages("erode2", src_left, src_right);
//
//    static Mat kernel_dilate2 = getStructuringElement(MORPH_RECT, Size(3, 6));
//    dilate(src_left, src_left, kernel_dilate2);
//    dilate(src_right, src_right, kernel_dilate2);
//    //showTwoImages("dilate2", src_left, src_right);

    float alpha = 1.5;
    int beta = 0;
    src_left.convertTo(src_left, -1, alpha, beta);
    src_right.convertTo(src_right, -1, alpha, beta);
    src_left -= 30;
    src_right -= 30;
    src_left *= 5;
    src_right *= 5;
    showTwoImages("enlighted", src_left, src_right);



    /************* debug code ***************************************/
    threshold(src_left, src_bin_left_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
    threshold(src_right, src_bin_right_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
    showTwoImages("bin", src_bin_left_, src_bin_right_);
    /*********************************************************/

    /************************** find light blobs **********************************************/
    light_blobs_left_.clear();
    light_blobs_right_.clear();
    bool state_left, state_right;
    //imshow("src_before", src_left);
    state_left = findLightBlob(src_left, light_blobs_left_);
    state_right = findLightBlob(src_right, light_blobs_right_);

//    replace_img(src_left, src_raw_left_, light_blobs_left_);
//    replace_img(src_right, src_raw_right_, light_blobs_right_);
//    imshow("src_after",src_left);



    if (!(state_left && state_right)) { return false; }
    showContours("light contours", src_left, light_blobs_left_, src_right, light_blobs_right_);



    /*************************** match light blobs***********************************/
//    vector<cv::Rect2d> left, right;
//    Mat left_show, right_show;
//    resize(src_raw_left_, left_show, Size(320, 240));
//    resize(src_raw_right_, right_show, Size(320, 240));
//    cvtColor(left_show, left_show, COLOR_GRAY2RGB);
//    cvtColor(right_show, right_show, COLOR_GRAY2RGB);
//    for (auto &a:light_blobs_left_)drawRotatedRectangle(left_show, a.rect);
//    for (auto &a:light_blobs_right_)drawRotatedRectangle(right_show, a.rect);
//    resize(right_show, right_show, Size(640, 480));
//    resize(left_show, left_show, Size(640, 480));
//
//    //showTwoImages("show", left_show, right_show);
//
//    state_left = matchLightBlobVector(light_blobs_left_, left);
//    state_right = matchLightBlobVector(light_blobs_right_, right);

    state_left = matchLightBlob(light_blobs_left_, armor_box_left_);
    state_right = matchLightBlob(light_blobs_right_, armor_box_right_);

    if(!(state_left && state_right)) {return false;}
    //
    //showArmorBoxVector("armor boxes", src_left, left, src_right, right);
//    if(left.empty() || right.empty())
//    {
//        return false;
//    } else{
//        armor_box_left_ = left.at(0);
//        armor_box_right_ = right.at(0);
//        total_contour_area_left_ = armor_box_left_.area();
//        total_contour_area_right_ = armor_box_right_.area();
//    }



    /********************** convert to 3d coordinate *********************************/
    convertToStereoscopicCoordinate(armor_box_left_, armor_box_right_, armor_space_position_);

    showArmorBox("armor box", src_left, armor_box_left_, src_right, armor_box_right_);


    /********************** convert 3d coordinate back to two camera vision ***************/
    //showSpacePositionBackToStereoVision(src_left, src_right, armor_space_position_);


    /******************** predict the armor moving path *******************************/
    //predictArmorPosition(armor_space_position_, armor_predicted_position_);



    /*********************** send position by uart **************************************/
    cout<<armor_space_position_<<endl;
//    armor_space_position_.x += 5;
//    armor_space_position_.y = armor_space_position_.y * 1.63 + 7.7;
//    armor_space_position_.z = armor_space_position_.z * 1.24 - 42.4;

    sendTargetByUart(
            static_cast<float>(armor_space_position_.x),
            static_cast<float>(armor_space_position_.y),
            static_cast<float>(armor_space_position_.z));

    return false;

}