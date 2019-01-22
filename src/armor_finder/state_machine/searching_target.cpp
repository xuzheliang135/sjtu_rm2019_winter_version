#include "armor_finder/armor_finder.h"
using namespace cv;
using namespace std;
//double leastSquare1(const LightBlob &light_blob) {
//    double x_average = 0, y_average = 0, x_squa_average = 0, x_y_average = 0;
//    for (auto &point:light_blob.contours) {
//        x_average += point.x;
//        y_average += point.y;
//        x_squa_average += point.x * point.x;
//        x_y_average += point.x * point.y;
//    }
//    x_average /= light_blob.contours.size();
//    y_average /= light_blob.contours.size();
//    x_squa_average /= light_blob.contours.size();
//    x_y_average /= light_blob.contours.size();
//    return (x_y_average - x_average * y_average) / (x_squa_average - x_average * x_average);
//
//}
//void drawLine(Mat &img,const LightBlob &light_blob,const Scalar &s){
//    int length=20;
//    double k=leastSquare1(light_blob);
//    Point2f center(light_blob.rect.center);
//    Point2f a(center),b(center);
//    a.x+=length;
//    b.x-=length;
//    a.y+=length*k;
//    b.y-=length*k;
//    line(img,a,b,s);
//}
void preprocess(Mat &src, int n, float percent) {
    src -= n;
    src *= percent;
    src -= n;
    src *= percent;
}

void ArmorFinder::imagePreprocess(cv::Mat &src_left, cv::Mat &src_right) {
    if (src_left.type() == CV_8UC1) {
        splitBayerBG(src_left, src_blue0, src_red0);
        splitBayerBG(src_right, src_blue1, src_red1);
        if (enemy_color_ == ENEMY_RED) {
            src_left_ = src_red0 - src_blue0;
            src_right_ = src_red1 - src_blue1;
        } else if (enemy_color_ == ENEMY_BLUE) {
            src_left_ = src_blue0 - src_red0;
            src_right_ = src_blue1 - src_red1;
        }

    } else if (src_left.type() == CV_8UC3) {
        std::vector<Mat> channels_left, channels_right;
        split(src_left, channels_left);
        split(src_right, channels_right);
        resize(channels_left.at(0), src_blue0, Size(SRC_WIDTH, SRC_HEIGHT));
        resize(channels_left.at(2), src_red0, Size(SRC_WIDTH, SRC_HEIGHT));
        resize(channels_right.at(0), src_blue1, Size(SRC_WIDTH, SRC_HEIGHT));
        resize(channels_right.at(2), src_red1, Size(SRC_WIDTH, SRC_HEIGHT));
        if (enemy_color_ == ENEMY_RED) {
            src_left_ = src_red0;
            src_right_ = src_red1;
        } else if (enemy_color_ == ENEMY_BLUE) {
            src_left_ = src_blue0;
            src_right_ = src_blue1;
        }
    }

}

void drawRotatedRectangle(Mat &img, const RotatedRect &rect, const Scalar &s) {
    Point2f points[4];
    rect.points(points);
    for (int i = 0; i < 3; i++)line(img, points[i], points[i + 1], s);
    line(img, points[3], points[0], s);
}

void ArmorFinder::clear_light_blobs_vector() {
    light_blobs_right_light.clear();
    light_blobs_left_light.clear();

    light_blobs_right_color.clear();
    light_blobs_left_color.clear();

    light_blobs_right_real.clear();
    light_blobs_left_real.clear();
}

void judge_light_color(vector<LightBlob> &light, vector<LightBlob> &color, vector<LightBlob> &result) {
    for (auto &i:color) {
        for (auto &j:light) {
            Rect2d a=i.rect.boundingRect2f();
            Rect2d b=j.rect.boundingRect2f();
            Rect2d ab=a&b;
            if(ab.area()/min(a.area(),b.area())>=0.2){
                result.emplace_back(j);
                break;
            }
        }
    }
}

void tmp(cv::Mat &src_left, cv::Mat &src_right) {
    static Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(1, 4));
    erode(src_left, src_left, kernel_erode);
    erode(src_right, src_right, kernel_erode);

    static Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(2, 4));
    dilate(src_left, src_left, kernel_dilate);
    dilate(src_right, src_right, kernel_dilate);

    static Mat kernel_erode2 = getStructuringElement(MORPH_RECT, Size(2, 4));
    erode(src_left, src_left, kernel_erode2);
    erode(src_right, src_right, kernel_erode2);

    static Mat kernel_dilate2 = getStructuringElement(MORPH_RECT, Size(3, 6));
    dilate(src_left, src_left, kernel_dilate2);
    dilate(src_right, src_right, kernel_dilate2);

    float alpha = 1.5;
    int beta = 0;
    src_left.convertTo(src_left, -1, alpha, beta);
    src_right.convertTo(src_right, -1, alpha, beta);
}

Mat ArmorFinder::getNumberPic(Mat &src, const Rect &rect) {
    return src(rect);
}

bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left_light, cv::Mat &src_right_light) {
    /************************** find light blobs **********************************************/
    src_raw_left_ = src_left_light.clone();
    src_raw_right_ = src_right_light.clone();

    imagePreprocess(src_left_light, src_right_light);

    preprocess(src_left_light, 150, 3.5);
    preprocess(src_right_light, 150, 3.5);
    tmp(src_left_, src_right_);//腐蚀，膨胀
    showTwoImages("color_after_erode", src_left_, src_right_);

    resize(src_left_, src_left_, Size(640, 480));
    resize(src_right_, src_right_, Size(640, 480));

    clear_light_blobs_vector();

    findLightBlob(src_left_light, light_blobs_left_light);
    findLightBlob(src_right_light, light_blobs_right_light);

    findLightBlob(src_left_, light_blobs_left_color);
    findLightBlob(src_right_, light_blobs_right_color);

    judge_light_color(light_blobs_left_light, light_blobs_left_color, light_blobs_left_real);
    judge_light_color(light_blobs_right_light, light_blobs_right_color, light_blobs_right_real);

    {
        showTwoImages("color", src_left_, src_right_);
        showTwoImages("light", src_left_light, src_right_light);

        threshold(src_left_, src_bin_left_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
        threshold(src_right_, src_bin_right_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
        cvtColor(src_bin_left_, src_bin_left_, COLOR_GRAY2RGB);
        cvtColor(src_bin_right_, src_bin_right_, COLOR_GRAY2RGB);
        for (auto &a:light_blobs_left_color)drawRotatedRectangle(src_bin_left_, a.rect, Scalar(0, 0, 255));
        for (auto &a:light_blobs_right_color)drawRotatedRectangle(src_bin_right_, a.rect, Scalar(0, 0, 255));
        for (auto &a:light_blobs_left_light)drawRotatedRectangle(src_bin_left_, a.rect, Scalar(0, 255, 0));
        for (auto &a:light_blobs_right_light)drawRotatedRectangle(src_bin_right_, a.rect, Scalar(0, 255, 0));
        showTwoImages("color_bin", src_bin_left_, src_bin_right_);

        threshold(src_left_light, src_bin_left_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
        threshold(src_right_light, src_bin_right_, light_blob_param_.GRAY_THRESH, 255, THRESH_BINARY);
        cvtColor(src_bin_left_, src_bin_left_, COLOR_GRAY2RGB);
        cvtColor(src_bin_right_, src_bin_right_, COLOR_GRAY2RGB);
        for (auto &a:light_blobs_left_light)drawRotatedRectangle(src_bin_left_, a.rect, Scalar(0, 0, 255));
        for (auto &a:light_blobs_right_light)drawRotatedRectangle(src_bin_right_, a.rect, Scalar(0, 0, 255));
        showTwoImages("light_bin", src_bin_left_, src_bin_right_);
    }//debug show

//    showContours("light contours", src_left_, light_blobs_left_color, src_right_, light_blobs_right_color);

    /*************************** match light blobs***********************************/
    {
        Mat left_show(480, 640, CV_8UC3), right_show(480, 640, CV_8UC3);
        left_show = Scalar(0, 0, 0);//src_raw_left_.clone();
        right_show = Scalar(0, 0, 0);//src_raw_right_.clone();
//        cvtColor(left_show, left_show, COLOR_GRAY2RGB);
//        cvtColor(right_show, right_show, COLOR_GRAY2RGB);
//        for (auto &a:light_blobs_left_light)drawRotatedRectangle(left_show, a.rect, Scalar(255, 0, 0));
//        for (auto &a:light_blobs_right_light)drawRotatedRectangle(right_show, a.rect, Scalar(255, 0, 0));
//        for (auto &a:light_blobs_left_color)drawRotatedRectangle(left_show, a.rect, Scalar(0, 255, 0));
//        for (auto &a:light_blobs_right_color)drawRotatedRectangle(right_show, a.rect, Scalar(0, 255, 0));
        for (auto &a:light_blobs_left_real)drawRotatedRectangle(left_show, a.rect, Scalar(0, 0, 255));
        for (auto &a:light_blobs_right_real)drawRotatedRectangle(right_show, a.rect, Scalar(0, 0, 255));
//        for (auto &a:light_blobs_left_real)drawLine(left_show, a, Scalar(0, 0, 255));
//        for (auto &a:light_blobs_right_real)drawLine(right_show, a, Scalar(0, 0, 255));
        showTwoImages("show_rotated_Rect", left_show, right_show);
    }//debug show

    vector<cv::Rect2d> left, right;
    matchLightBlobVector(light_blobs_left_real, left);
    matchLightBlobVector(light_blobs_right_real, right);
    showArmorBoxVector("armor boxes", src_left_light, left, src_right_light, right);


    return false;

}