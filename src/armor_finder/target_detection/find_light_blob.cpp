
#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::initLightParam() {
    light_blob_param_.GRAY_THRESH = 240;
    light_blob_param_.CONTOUR_AREA_MIN = 20;
    light_blob_param_.CONTOUR_AREA_MAX = 3000;
    light_blob_param_.CONTOUR_LENGTH_MIN = 10;
    light_blob_param_.CONTOUR_HW_RATIO_MIN = 2.5;       // 2.5
    light_blob_param_.CONTOUR_HW_RATIO_MAX = 15;
    light_blob_param_.CONTOUR_ANGLE_MAX = 20.0;
}


bool ArmorFinder::findLightBlob(const cv::Mat &src, vector<LightBlob> &light_blobs) {
    static Mat src_bin;
    threshold(src, src_bin, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
    //imshow("binary image", src_bin);
    std::vector<vector<Point> > light_contours;
    findContours(src_bin, light_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
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
    if(cur_contour_area > light_blob_param_.CONTOUR_AREA_MAX ||
       cur_contour_area < light_blob_param_.CONTOUR_AREA_MIN)
    {
        //cout<<cur_contour_area<<" "<<light_blob_param_.CONTOUR_AREA_MIN<<" "<<light_blob_param_.CONTOUR_AREA_MAX<<endl;
        //cout<<"area fail."<<endl;
        return false;
    }
    RotatedRect cur_rect = minAreaRect(light_contour);
    Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if(length < light_blob_param_.CONTOUR_LENGTH_MIN)
    {
        //cout<<"length min fail."<<endl;
        return false;
    }
    float length_width_ratio = length / width;
    if(length_width_ratio > light_blob_param_.CONTOUR_HW_RATIO_MAX ||
       length_width_ratio < light_blob_param_.CONTOUR_HW_RATIO_MIN)
    {
        //cout<<"length width ratio fail."<<endl;
        return false;
    }
    if(cur_contour_area / cur_size.area() < 0.7) return false;
    return true;
}