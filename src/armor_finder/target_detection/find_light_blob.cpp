
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

void ArmorFinder::initLightParam() {
    light_blob_param_.GRAY_THRESH = 100;
    light_blob_param_.CONTOUR_AREA_MIN = 10;
    light_blob_param_.CONTOUR_AREA_MAX = 300;
    light_blob_param_.CONTOUR_LENGTH_MIN = 3;
    light_blob_param_.CONTOUR_HW_RATIO_MIN = 2.5;       // 2.5
    light_blob_param_.CONTOUR_HW_RATIO_MAX = 15;
    light_blob_param_.CONTOUR_ANGLE_MAX = 20.0;
    light_blob_param_.Y_POSITION_MIN = 50;
}


bool ArmorFinder::findLightBlob(const cv::Mat &src, vector<LightBlob> &light_blobs) {
    static Mat src_gray;
    static Mat src_bin;
    if(src.type() == CV_8UC3){
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
    }else if(src.type() == CV_8UC1){
        src_gray = src.clone();
    }
//    GaussianBlur(src_gray, src_gray, Size(3,3), 5, 5);
//
//    imshow("src_gray blue", src_gray);
//
    threshold(src_gray, src_bin, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
//
//    erode(src_bin, src_bin, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
//
//    imshow("src_bin erode", src_bin);


    std::vector<vector<Point> > light_contours;
    findContours(src_bin, light_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (auto &light_contour : light_contours) {
        if(!isValidLightContour(light_contour))
        {
            continue;
        }
        light_blobs.emplace_back(light_contour);
    }
    return light_blobs.size() >= 2;
}

bool ArmorFinder::isValidLightContour(const vector<Point> &light_contour) {
    double cur_contour_area = contourArea(light_contour);

    if ( cur_contour_area > light_blob_param_.CONTOUR_AREA_MAX ||
             cur_contour_area < light_blob_param_.CONTOUR_AREA_MIN){
        //cout<<cur_contour_area<<endl;
        return false;
    }

    if(light_contour[0].y < light_blob_param_.Y_POSITION_MIN )
    {
//        cout<<""<<endl;
        return false;
    }
    return true;
//    RotatedRect cur_rect = minAreaRect(light_contour);
//    Size2f cur_size = cur_rect.size;
//    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
//    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
//    if(length < light_blob_param_.CONTOUR_LENGTH_MIN)
//    {
//        //cout<<"length min fail."<<endl;
//        return false;
//    }
//    float length_width_ratio = length / width;
//    if(length_width_ratio > light_blob_param_.CONTOUR_HW_RATIO_MAX ||
//       length_width_ratio < light_blob_param_.CONTOUR_HW_RATIO_MIN)
//    {
//        //cout<<"length width ratio fail."<<endl;
//        return false;
//    }
//    if(cur_contour_area / cur_size.area() < 0.7) return false;
}