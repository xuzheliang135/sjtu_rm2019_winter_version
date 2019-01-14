
#include "armor_finder/armor_finder.h"

using namespace cv;


void ArmorFinder::showArmorBox(std::string windows_name,
                                const cv::Mat &src_left, const cv::Rect2d &armor_box_left,
                                const cv::Mat &src_right, const cv::Rect2d &armor_box_right)
{
    static Mat image2show_left, image2show_right;
    cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
    cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols+10, image2show_left.type());
    rectangle(image2show_left, armor_box_left, Scalar(0, 255, 0), 2);
    rectangle(image2show_right, armor_box_right, Scalar(0, 255, 0), 2);
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols+10, combined_image.cols));
    imshow(windows_name, combined_image);
}

void ArmorFinder::showContours(std::string windows_name,
                                const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left,
                                const cv::Mat &src_right, const std::vector<LightBlob> &light_blobs_right)
{
    static Mat image2show_left, image2show_right;
    cvtColor(src_left, image2show_left, COLOR_GRAY2RGB);
    cvtColor(src_right, image2show_right, COLOR_GRAY2BGR);
    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols+10, image2show_left.type());
    for(const auto &light_blob:light_blobs_left)
    {
        rectangle(image2show_left, light_blob.rect.boundingRect(), Scalar(255,0,0), 2);
    }
    for(const auto &light_blob:light_blobs_right)
    {
        rectangle(image2show_right, light_blob.rect.boundingRect(), Scalar(255,0,0), 2);
    }
    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols+10, combined_image.cols));
    imshow(windows_name, combined_image);


}


void ArmorFinder::showSpacePositionBackToStereoVision(const cv::Mat &src_left, const cv::Mat &src_right,
                                                      const cv::Point3d &space_position) {
    static Mat image2show_left, image2show_right;
    image2show_left = src_left.clone();
    image2show_right = src_right.clone();
    Mat combined_image(image2show_left.rows, image2show_left.cols+image2show_right.cols+10, image2show_left.type());

    cvtColor(image2show_left, image2show_left, COLOR_GRAY2RGB);
    cvtColor(image2show_right, image2show_right, COLOR_GRAY2RGB);
    cvtColor(combined_image, combined_image, COLOR_GRAY2RGB);

    Point2d left_center, right_center;
    left_center.x = space_position.x / space_position.z * stereo_camera_param_.FOCUS  / stereo_camera_param_.LENGTH_PER_PIXAL + 320;
    left_center.y = space_position.y / space_position.z * stereo_camera_param_.FOCUS  / stereo_camera_param_.LENGTH_PER_PIXAL + 240;

    double disparity = stereo_camera_param_.CAMERA_DISTANCE * stereo_camera_param_.FOCUS / space_position.z;
    right_center.x = left_center.x + disparity / stereo_camera_param_.LENGTH_PER_PIXAL;
    right_center.y = left_center.y;

    circle(image2show_left, left_center, 2, Scalar(0, 255, 0), 4);
    circle(image2show_right, right_center, 2, Scalar(0, 255, 0), 4);

    image2show_left.colRange(0, image2show_left.cols).copyTo(combined_image.colRange(0, image2show_left.cols));
    image2show_right.colRange(0, image2show_right.cols).copyTo(combined_image.colRange(image2show_left.cols+10, combined_image.cols));

    frame_to_display = src_left.clone();
    putText(frame_to_display, std::to_string(space_position.z), left_center,cv::FONT_HERSHEY_TRIPLEX, 0.8, cv::Scalar(255, 200, 200), 1);

    imshow("Reconstruct", combined_image);
}