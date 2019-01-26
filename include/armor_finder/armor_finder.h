//
// Created by zhikun on 18-11-11.
// this file used for testing detecting armor by light blob
// testing armor is large armor
//

#ifndef STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H
#define STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H

#include <algorithm>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <armor_finder/param_struct_define.h>
#include "armor_finder/constant.h"
#include <uart/uart.h>
#include <tracker/kcftracker.hpp>
#include "tracker/tracker.h"
#include "predictor/predictor_curve_fitting.h"

using std::vector;

/**
 * @brief   a class to find armor in image. SJTU-TPP@RoboMaster2019
 */
class ArmorFinder {
public:
    ArmorFinder();

    ~ArmorFinder()= default;

    /**
     *
     * @param src_left : input
     * @param src_right : input
     * @return : bool value: whether it success.
     */
    cv::Mat src_left_, src_right_;

    int run(cv::Mat &src_left, cv::Mat &src_right);

public:
    cv::Mat frame_to_display;
private:
    LightBlobParam light_blob_param_;
    LightCoupleParam light_couple_param_;
    StereoCameraPara stereo_camera_param_;
    ArmorSeekingParam armor_seeking_param_;
    ArmorPridictParam armor_predict_param_;
    StateMachineParam state_machine_param_;
    CalibrateParam calibrate_param_;
    TrackingParam track_param_;

    std::vector<LightBlob> light_blobs_left_light_, light_blobs_right_light_;
    std::vector<LightBlob> light_blobs_left_color_, light_blobs_right_color_;
    std::vector<LightBlob> light_blobs_left_real_, light_blobs_right_real_;
    std::vector<int> armor_num_left, armor_num_right;

    cv::Rect2d armor_box_left_, armor_box_right_;

    std::vector<cv::Rect2d> armor_boxes_left_, armor_boxes_right_;

    int target_found_frame_cnt, target_unfound_frame_cnt;

    StateMachine cur_state_;


    Uart uart_;

    KCFTracker kcf_tracker_left_, kcf_tracker_right_;

    cv::Mat src_blue0, src_red0, src_blue1, src_red1;
    cv::Mat src_raw_right_, src_raw_left_;
    cv::Mat src_bin_left_, src_bin_right_;

    int enemy_color_;

    double total_contour_area_right_;
    double total_contour_area_left_;


    cv::Point3d armor_space_position_;
    cv::Point3d armor_space_last_position_;
    std::vector<cv::Point3d> armor_history_positions_;
    cv::Point3d armor_predicted_position_;
    std::vector<clock_t> time_serial;
    double position_diff;




public:
    void setEnemyColor(int color);
    void calibrate(cv::Mat &src_left, cv::Mat &src_right);

private:

    void initCalibrateParam();

    void initLightParam();

    void initLightCoupleParam();

    void initCameraParam();

    void initArmorSeekingParam();

    void initArmorPredictParam();

    void initUartParam();

    void initStateMachineParam();

    void initTrackingParam();

    void transferState(StateMachine state);

    bool stateStandBy();

    bool stateSearchingTarget(cv::Mat &src_left, cv::Mat &src_right);

    bool stateTrackingTarget(cv::Mat &src_left, cv::Mat &src_right);

    void splitBayerBG(cv::Mat &src, cv::Mat &blue, cv::Mat &red);

    void imagePreprocess(cv::Mat &src_left, cv::Mat &src_right, cv::Mat &src_output_left, cv::Mat &src_output_right);

    bool pipelineForFindLightBlob(cv::Mat &src_left, cv::Mat &src_right, std::vector<LightBlob> &light_blobs_real_left, std::vector<LightBlob> &light_blobs_real_right);

    void pipelineLightBlobPreprocess(cv::Mat &InOutput);

    bool pipelineTargetPosition(cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right);

    int recognize_digits(cv::Mat &image);

    bool targetPositionStreamControl(float x, float y);

public:
    bool matchTwoArmorBox(vector<cv::Rect2d> &armor_box_list_left, vector<cv::Rect2d> &armor_box_list_right,
            cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right);

public:

    void clear_light_blobs_vector();


public:
    /**
     * @name    findLightBlob()
     * @brief   Find out all the light blobs in given image
     * @param   src: input image
     * @param   light_blobs: output vector of light blobs
     * @return  bool value: whether it finds more than 2 valid light blobs
     */
    bool findLightBlob(const cv::Mat &src, std::vector<LightBlob> &light_blobs);

public:


    bool matchLightBlobVector(std::vector<LightBlob> &light_blobs, vector<cv::Rect2d> &armor_box);

public:
    /**
     * @brief   convert two 2D coordinates to 3D coordinate
     * @param   armor_box_left: input armor box rect in left camera
     * @param   armor_box_right: input armor box rect in right camera
     * @param   space_position: output 3D point of armor
     * @return  bool value: whether it can be converted
     */
    bool convertToStereoscopicCoordinate(
            cv::Rect2d &armor_box_left, cv::Rect2d &armor_box_right, cv::Point3d &space_position);

public:
    /**
     *
     * @param armor_history_position
     * @param armor_predicted_position
     * @return
     */
    bool predictArmorPosition(
            cv::Point3d &armor_history_position, cv::Point3d &armor_predicted_position);


public:
    bool sendTargetByUart(float x, float y, float z);

private:
    /**
     * @name    isValidLightContour()
     * @brief   judge a contour whether valid or not
     * @param   light_contour: input light contour
     * @return  bool value: whether the light contour is valid
     */
    bool isValidLightContour(const vector<cv::Point> &light_contour);


    /**
     * @name    isCoupleLight()
     * @brief   judge two light blobs are a couple or not
     * @param   light_blob_x: input light blob x
     * @param   light_blob_y: input light blob y
     * @return  bool value: whether the two light blob is a couple
     */
    bool isCoupleLight(const LightBlob &light_blob_x, const LightBlob &light_blob_y);


    void manageHistorySpacePosition(const cv::Point3d &space_position);

public:
    void showTwoImages(std::string windows_name, const cv::Mat &src0, const cv::Mat &src1);

    void showContours(std::string windows_name, const cv::Mat &src_left, const std::vector<LightBlob> &light_blobs_left,
            const cv::Mat &src_right, const std::vector<LightBlob> &light_blobs_right);


    void showArmorBox(std::string windows_name, const cv::Mat &src_left, const cv::Rect2d &armor_box_left,
            const cv::Mat &src_right, const cv::Rect2d &armor_box_right);

    void showArmorBoxVector(std::string windows_name, const cv::Mat &src_left, const vector<cv::Rect2d> &armor_box_left,
                            const cv::Mat &src_right, const vector<cv::Rect2d> &armor_box_right);

    void showSpacePositionBackToStereoVision(
            const cv::Mat &src_left, const cv::Mat &src_right, const cv::Point3d &space_position);

    void trackInit(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box);

    bool track(KCFTracker &kcf_tracker, cv::Mat &src, cv::Rect2d &armor_box);

};


#endif //STEREOVISION_FROM_VIDEO_FILE_ARMOR_FINDER_H
