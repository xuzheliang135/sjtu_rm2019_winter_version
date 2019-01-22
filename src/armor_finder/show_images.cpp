
#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::showTwoImages(std::string windows_name, const cv::Mat &src_left, const cv::Mat &src_right){
    static Mat image2show_left, image2show_right;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);

    } else if (src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_right = src_right.clone();
        image2show_left = src_left.clone();

    }
    Mat combined_image(image2show_left.rows, image2show_left.cols + image2show_right.cols, image2show_left.type());
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(
            combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showArmorBoxVector(std::string windows_name,
                                     const cv::Mat &src_left, const vector<cv::Rect2d> &armor_box_left,
                                     const cv::Mat &src_right, const vector<cv::Rect2d> &armor_box_right) {
    static Mat image2show_left, image2show_right;
    if (src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    } else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }
    Mat combined_image(image2show_left.rows, image2show_left.cols + image2show_right.cols, image2show_left.type());
    for (auto &a:armor_box_left) {
        rectangle(image2show_left, a, Scalar(0, 255, 0), 1);

    }
    for (auto &a:armor_box_right) {
        rectangle(image2show_right, a, Scalar(0, 255, 0), 1);
    }
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showArmorBox(std::string windows_name,
                                const cv::Mat &src_left, const cv::Rect2d &armor_box_left,
                                const cv::Mat &src_right, const cv::Rect2d &armor_box_right)
{
    static Mat image2show_left, image2show_right;
    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }
    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols, image2show_left.type());
    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 1);
    rectangle(image2show_right, armor_box_right, Scalar(0, 255, 0), 1);
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showContours(std::string windows_name,
                                const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left,
                                const cv::Mat &src_right, const std::vector<LightBlob> &light_blobs_right)
{
    static Mat image2show_left, image2show_right;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
        cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
        image2show_right = src_right.clone();
    }

    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols, image2show_left.type());
    for(const auto &light_blob:light_blobs_left)
    {
        rectangle(image2show_left, light_blob.rect.boundingRect(), Scalar(255,0,0), 1);
    }
    for(const auto &light_blob:light_blobs_right)
    {
        rectangle(image2show_right, light_blob.rect.boundingRect(), Scalar(255,0,0), 1);
    }
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols, combined_image.cols));
    imshow(windows_name, combined_image);


}