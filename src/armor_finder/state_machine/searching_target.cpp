#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;




bool ArmorFinder::stateSearchingTarget(cv::Mat &src_left_light, cv::Mat &src_right_light) {

    /************************** find light blobs **********************************************/
    imagePreprocess(src_left_light, src_right_light, src_left_, src_right_);  // bayer hacking, to split blue and red

    pipelineForFindLightBlob(src_left_light, src_right_light, light_blobs_left_real_, light_blobs_right_real_);

    /*************************** match light blobs***********************************/

    matchLightBlobVector(light_blobs_left_real_, armor_boxes_left_);
    matchLightBlobVector(light_blobs_right_real_, armor_boxes_right_);
    //showArmorBoxVector("armor boxes", src_raw_left_, armor_boxes_left_, src_raw_right_, armor_boxes_right_);


    bool state_match = matchTwoArmorBox(armor_boxes_left_, armor_boxes_right_, armor_box_left_, armor_box_right_);
    if(!state_match) {return false;}



    /********************** convert to 3d coordinate and send it by uart *********************************/
return pipelineTargetPosition(armor_box_left_, armor_box_right_);


}