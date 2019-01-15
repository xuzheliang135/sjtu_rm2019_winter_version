
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

int ArmorFinder::run(cv::Mat &src_left, cv::Mat &src_right) {

    splitBayerBG(src_left, src_blue0, src_red0);
    splitBayerBG(src_right, src_blue1, src_red1);

    if(enemy_color_ == ENEMY_RED)
    {
        src_left_ = src_red0 - src_blue0;
        src_right_ = src_red1 - src_blue1;
    }else if(enemy_color_ == ENEMY_BLUE){
        src_left_ = src_blue0 - src_red0;
        src_right_ = src_blue1 - src_red1;
    }

    if(cur_state_ == STAND_BY)
    {
        //stateStandBy();
        transferState(SEARCHING_TARGET);
    }
    else if(cur_state_ == SEARCHING_TARGET)
    {
        if(stateSearchingTarget(src_left_, src_right_))
        {
            target_found_frame_cnt++;
            target_unfound_frame_cnt = 0;
        } else{
            target_unfound_frame_cnt++;
            target_found_frame_cnt = 0;
        }
        if(target_found_frame_cnt > state_machine_param_.THRESHOLD_FOUND_SEARCHING_TO_TRACKING){
            trackInit(kcf_tracker_left_, src_left_, armor_box_left_);
            trackInit(kcf_tracker_right_, src_right_, armor_box_right_);
            transferState(TRACKING_TARGET);
        }
    }
    else if(cur_state_ == TRACKING_TARGET)
    {
        if(stateTrackingTarget(src_left_, src_right_))
        {
            target_found_frame_cnt++;
            target_unfound_frame_cnt = 0;
        }else {
            target_unfound_frame_cnt++;
            target_found_frame_cnt = 0;
        }
        if(target_unfound_frame_cnt > state_machine_param_.THRESHOLD_UNFOUND_TRACKING_TO_SEARCHING){
            transferState(SEARCHING_TARGET);
        }
    }

    return 0;
}