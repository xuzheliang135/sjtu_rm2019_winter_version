
#include <armor_finder/armor_finder.h>

using namespace cv;
using std::cout;
using std::endl;
using std::vector;


ArmorFinder::ArmorFinder():
                kcf_tracker_left_(false, true, false, false),
                kcf_tracker_right_(false, true, false, false)
                {
    initLightParam();
    initLightCoupleParam();
    initCameraParam();
    initArmorSeekingParam();
    initArmorPredictParam();
    initUartParam();

    cur_state_ = SEARCHING_TARGET;
    target_found_frame_cnt = 0;
    target_unfound_frame_cnt = 0;
}


