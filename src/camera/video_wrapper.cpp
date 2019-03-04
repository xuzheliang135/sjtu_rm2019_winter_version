//
// Created by zhikun on 18-11-16.
//

#include "camera/video_wrapper.h"


VideoWrapper::VideoWrapper(const std::string &filename0) {
    video0.open(filename0);
}

VideoWrapper::~VideoWrapper() = default;


bool VideoWrapper::init() {
    return video0.isOpened();
}

bool VideoWrapper::read(cv::Mat &src_left) {
    return video0.read(src_left);




}
