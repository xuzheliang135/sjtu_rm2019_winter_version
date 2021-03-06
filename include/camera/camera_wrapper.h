//
// Created by zhikun on 18-11-7.
// used for testing double cameras
// camera0 is left camera, camera1 is right camera.
//

#ifndef VIDEO_TEST1_CAMERA_WRAPPER_H
#define VIDEO_TEST1_CAMERA_WRAPPER_H

#include <stdio.h>
#include <iostream>
#include <thread>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include "wrapper_head.h"
#include "camera_api.h"

class CameraWrapper: public WrapperHead {
private:
    unsigned char* rgb_buffer0;
    int camera_cnts;
    int camera_status;
    tSdkCameraDevInfo camera_enum_list[2];
    int h_camera;
    char camera_name[32];

    tSdkCameraCapbility tCapability;
    tSdkFrameHead frame_info;
    BYTE *pby_buffer;
    IplImage *iplImage;
    int channel;
    bool read_state0;

public:
    CameraWrapper();
    ~CameraWrapper() final;

    /**
     * @brief initialize the cameras, including connecting devices, setting handle and so on
     * @return
     */
    bool init() final;

    /**
     * @brief read image from cameras,
     * @param src0
     * @param src1
     * @return
     */
    bool read(cv::Mat &src0) final;

    /**
     * @brief read the image without process, it is a single channel, but it is a bayer matrix
     * @param src0
     * @param src1
     * @return
     */
    bool readRaw(cv::Mat &src0);

    /**
     * @brief read the image with process, it is three channels color image, but it is slower.
     * @param src0
     * @param src1
     * @return
     */
    bool readProcessed(cv::Mat& src0, cv::Mat& src1);

    /**
     * @brief try to read the camera image in two thread, (but it seems that the reading is already implemented in thread.)
     * @param src0
     * @param src1
     * @return
     */
    bool read_thread(cv::Mat& src0, cv::Mat& src1);
    void read_camera_thread0(cv::Mat &src);

};


#endif //VIDEO_TEST1_CAMERA_WRAPPER_H
