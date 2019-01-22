
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

int ArmorFinder::run(cv::Mat &src_left, cv::Mat &src_right) {
    src_raw_left_ = src_left.clone();
    src_raw_right_ = src_right.clone();

    //showTwoImages("before split",src_raw_left_, src_raw_right_);
    imagePreprocess(src_left, src_right);   // to split blue and red
    //showTwoImages("after split ", src_left_, src_right_);


    switch (cur_state_){
        case SEARCHING_TARGET:
            if(stateSearchingTarget(src_left_, src_right_))
            {
                //target_found_frame_cnt++;
            } else{
                target_found_frame_cnt = 0;
            }
            if(target_found_frame_cnt > state_machine_param_.THRESHOLD_FOUND_SEARCHING_TO_TRACKING){
                trackInit(kcf_tracker_left_, src_left_, armor_box_left_);
                trackInit(kcf_tracker_right_, src_right_, armor_box_right_);
                transferState(TRACKING_TARGET);
                std::cout<<"dive into tracking"<<std::endl;
            }
            break;
        case TRACKING_TARGET:
            if(!stateTrackingTarget(src_left_, src_right_))
            {
                std::cout<<"jump out tracking"<<endl;
                transferState(SEARCHING_TARGET);
            }
            break;
        case STAND_BY:
            stateStandBy();
            transferState(SEARCHING_TARGET);
            break;
        default:
            cout<<"incorrect state: "<<cur_state_<<endl;
    }

        return 0;
}