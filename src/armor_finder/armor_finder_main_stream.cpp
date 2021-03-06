
#include "armor_finder/armor_finder.h"

using namespace cv;
using std::cout;
using std::endl;

int ArmorFinder::run(cv::Mat &src_left) {
    src_raw_left_ = src_left.clone();
    if(src_raw_left_.type() == CV_8UC3)
    {
        cvtColor(src_raw_left_, src_raw_left_, CV_RGB2GRAY);
    }


    switch (cur_state_) {
        case SEARCHING_TARGET:
            if (stateSearchingTarget(src_left)) {
                target_found_frame_cnt++;
            } else {
                target_found_frame_cnt = 0;
            }
            if (target_found_frame_cnt >= state_machine_param_.THRESHOLD_FOUND_SEARCHING_TO_TRACKING) {
                if ((Rect2d(0, 0, 640, 480) & armor_box_left_).area() <
                    armor_box_left_.area()) { // avoid box touching edges
                    target_found_frame_cnt = 0;
                    break;
                }
                Mat roi_left = src_raw_left_.clone()(armor_box_left_);
                threshold(roi_left, roi_left, track_param_.THRESHOLD_FOR_COUNT_NON_ZERO, 255, THRESH_BINARY);
                total_contour_area_left_ = countNonZero(roi_left);

                trackInit(kcf_tracker_left_, src_raw_left_, armor_box_left_);
                transferState(TRACKING_TARGET);
                //std::cout<<"dive into tracking"<<std::endl;

            }
            break;

        case TRACKING_TARGET:
            if (!stateTrackingTarget(src_raw_left_)) {
                //std::cout << "jump out tracking" << endl;
                transferState(SEARCHING_TARGET);
            }
            break;
        case STAND_BY:

            stateStandBy();
            transferState(SEARCHING_TARGET);
            break;
        default:
            cout << "incorrect state: " << cur_state_ << endl;
    }

    return 0;
}