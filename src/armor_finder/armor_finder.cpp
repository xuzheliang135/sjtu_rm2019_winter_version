
#include <armor_finder/armor_finder.h>

using namespace cv;
using std::cout;
using std::endl;
using std::vector;


ArmorFinder::ArmorFinder():
                kcf_tracker_left_(false, true, false, false),
                kcf_tracker_right_(false, true, false, false),
                src_blue0(SRC_HEIGHT, SRC_WIDTH, CV_8UC1),
                src_blue1(SRC_HEIGHT, SRC_WIDTH, CV_8UC1),
                src_red0(SRC_HEIGHT, SRC_WIDTH, CV_8UC1),
                src_red1(SRC_HEIGHT, SRC_WIDTH, CV_8UC1)
                {
    initLightParam();
    initLightCoupleParam();
    initCameraParam();
    initArmorSeekingParam();
    initArmorPredictParam();
    initUartParam();
    initStateMachineParam();

    cur_state_ = SEARCHING_TARGET;
    target_found_frame_cnt = 0;
    target_unfound_frame_cnt = 0;
}


void ArmorFinder::setEnemyColor(int color)
{
    enemy_color_ = color;
}

void ArmorFinder::splitBayerBG(cv::Mat &src, cv::Mat &blue, cv::Mat &red) {
    uchar* data;
    uchar* bayer_data[2];
    for (int i = 0; i < src.rows; ++i) {
        data = src.ptr<uchar>(i);
        bayer_data[0] = blue.ptr<uchar>(i / 2);
        for (int j = 0; j < blue.cols; ++j, data += 2) {
            bayer_data[0][j] = *data;
        }
        data = src.ptr<uchar>(++i) + 1;
        bayer_data[1] = red.ptr<uchar>(i / 2);
        for (int j = 0; j < red.cols; ++j, data += 2) {
            bayer_data[1][j] = *data;
        }
    }

}