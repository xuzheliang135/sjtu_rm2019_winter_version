//
// Created by zhikun on 18-11-16.
// wrapper for video read from file
//

#ifndef STEREOVISION_FROM_VIDEO_FILE_VIDEO_WRAPPER_H
#define STEREOVISION_FROM_VIDEO_FILE_VIDEO_WRAPPER_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "wrapper_head.h"

/**
 * @brief a class to read images from video files
 */
class VideoWrapper:public WrapperHead {
public:
    /**
     * @brief open two video files
     * @param filename0
     * @param filename1
     */
    VideoWrapper(const std::string &filename0);
    ~VideoWrapper();


    /**
     * @brief
     * @return bool value: whether it success
     */
    bool init() final;


    /**
     * @brief read images from video files
     * @param src_left : output source video of left camera
     * @param src_right : output source video of right camera
     * @return bool value: whether the reading is successful
     */
    bool read(cv::Mat &src_left) final;
private:
    cv::VideoCapture video0;

};


#endif //STEREOVISION_FROM_VIDEO_FILE_VIDEO_WRAPPER_H
