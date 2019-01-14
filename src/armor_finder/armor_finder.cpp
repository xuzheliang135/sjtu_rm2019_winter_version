
#include <armor_finder/armor_finder.h>

using namespace cv;
using std::cout;
using std::endl;
using std::vector;


ArmorFinder::ArmorFinder():
                kcf_tracker_(false, true, false, false)
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


void ArmorFinder::initArmorSeekingParam() {
    armor_seeking_param_.BORDER_IGNORE = 10;
    armor_seeking_param_.BOX_EXTRA = 10;
}