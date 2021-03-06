
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

void ArmorFinder::initLightParam() {
    light_blob_param_.PREPROCESS_SUBSTRACT_FACTOR = 150;
    light_blob_param_.PREPROCESS_MULTIPLY_FACTOR = 3.5;
    light_blob_param_.GRAY_THRESH = 80;
    light_blob_param_.CONTOUR_AREA_MIN = 1;
    light_blob_param_.CONTOUR_AREA_MAX = 3000;
    light_blob_param_.CONTOUR_LENGTH_MIN = 3;
    light_blob_param_.CONTOUR_HW_RATIO_MIN = 2.5;       // 2.5
    light_blob_param_.CONTOUR_HW_RATIO_MAX = 15;
    light_blob_param_.CONTOUR_ANGLE_MAX = 20.0;
    light_blob_param_.Y_POSITION_MIN = 50;
}



void ArmorFinder::pipelineLightBlobPreprocess(Mat &src) {
    src -= light_blob_param_.PREPROCESS_SUBSTRACT_FACTOR;
    src *= light_blob_param_.PREPROCESS_MULTIPLY_FACTOR;
    src -= light_blob_param_.PREPROCESS_SUBSTRACT_FACTOR;
    src *= light_blob_param_.PREPROCESS_MULTIPLY_FACTOR;
}

void drawRotatedRectangle(Mat &img, const RotatedRect &rect, const Scalar &s) {
    Point2f points[4];
    rect.points(points);
    for (int i = 0; i < 3; i++)line(img, points[i], points[i + 1], s);
    line(img, points[3], points[0], s);
}

void ArmorFinder::clear_light_blobs_vector() {
    light_blobs_left_light_.clear();

    light_blobs_left_color_.clear();

    light_blobs_left_real_.clear();
}

void judge_light_color(vector<LightBlob> &light, vector<LightBlob> &color, vector<LightBlob> &result) {
    for (auto &i:color) {
        for (auto &j:light) {
            Rect2d a = i.rect.boundingRect2f();
            Rect2d b = j.rect.boundingRect2f();
            Rect2d ab = a & b;
            if (ab.area() / min(a.area(), b.area()) >= 0.2) {
                result.emplace_back(j);
                break;
            }
        }
    }
}

void preprocessColor(cv::Mat &src_left) {
    static Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(1, 4));
    erode(src_left, src_left, kernel_erode);

    static Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(2, 4));
    dilate(src_left, src_left, kernel_dilate);

    static Mat kernel_erode2 = getStructuringElement(MORPH_RECT, Size(2, 4));
    erode(src_left, src_left, kernel_erode2);

    static Mat kernel_dilate2 = getStructuringElement(MORPH_RECT, Size(3, 6));
    dilate(src_left, src_left, kernel_dilate2);

    float alpha = 1.5;
    int beta = 0;
    src_left.convertTo(src_left, -1, alpha, beta);
}

bool ArmorFinder::findLightBlob(const cv::Mat &src, vector<LightBlob> &light_blobs) {
    static Mat src_gray;
    static Mat src_bin;
    if(src.type() == CV_8UC3){
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
    }else if(src.type() == CV_8UC1){
        src_gray = src.clone();
    }

    threshold(src_gray, src_bin, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);

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
    return !(cur_contour_area > light_blob_param_.CONTOUR_AREA_MAX ||
             cur_contour_area < light_blob_param_.CONTOUR_AREA_MIN);
}

bool ArmorFinder::pipelineForFindLightBlob(cv::Mat &src_left_light,
                                           std::vector<LightBlob> &light_blobs_real_left) {

    pipelineLightBlobPreprocess(src_left_light);

    preprocessColor(src_left_); //腐蚀，膨胀
//    showImage("small size", src_left_, src_right_);
//    showImage("color_after_erode", src_left_, src_right_);

    resize(src_left_, src_left_, Size(640, 480));

    clear_light_blobs_vector();

    findLightBlob(src_left_light, light_blobs_left_light_);
//    showContours("lightbolbs light", src_left_light, light_blobs_left_light_, src_right_light, light_blobs_right_light_);

    findLightBlob(src_left_, light_blobs_left_color_);
//    showContours("lightbolbs color", src_left_, light_blobs_left_color_, src_right_, light_blobs_right_color_);



    judge_light_color(light_blobs_left_light_, light_blobs_left_color_, light_blobs_real_left);

//    showContours("light blobs real", src_raw_left_, light_blobs_real_left, src_raw_right_, light_blobs_real_right);

    return !(light_blobs_real_left.empty());
}
