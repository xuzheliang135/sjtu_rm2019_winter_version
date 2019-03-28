
#include "armor_finder/armor_finder.h"

using namespace cv;

void ArmorFinder::showImage(std::string windows_name, const cv::Mat &src_left) {
    static Mat image2show_left;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);

    } else if (src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();

    }
    imshow(windows_name, image2show_left);
}

void ArmorFinder::showArmorBoxVector(std::string windows_name,
                                     const cv::Mat &src_left, const vector<cv::Rect2d> &armor_box_left) {
    static Mat image2show_left;
    if (src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
    } else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
    }
    for (auto &a:armor_box_left) {
        rectangle(image2show_left, a, Scalar(0, 255, 0), 1);

    }
    imshow(windows_name, image2show_left);
}

void ArmorFinder::showArmorBox(std::string windows_name,
                               const cv::Mat &src_left, const cv::Rect2d &armor_box_left) {
    static Mat image2show_left;
    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        std::cout<<" is 8uc3"<<std::endl;
        image2show_left = src_left.clone();
    }
    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 3);
    imshow(windows_name, image2show_left);

    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 1);
    imshow(windows_name, image2show_left);
}

void ArmorFinder::showContours(std::string windows_name,
                               const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left) {
    static Mat image2show_left;

    if(src_left.type() == CV_8UC1) // 黑白图像
    {
        cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
    }
    else if(src_left.type() == CV_8UC3) //RGB 彩色
    {
        image2show_left = src_left.clone();
    }

    for(const auto &light_blob:light_blobs_left)
    {
        rectangle(image2show_left, light_blob.rect.boundingRect(), Scalar(255,0,0), 3);
    }
    imshow(windows_name, image2show_left);


}
