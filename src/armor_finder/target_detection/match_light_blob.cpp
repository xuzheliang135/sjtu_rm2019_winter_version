
#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"

using namespace cv;


void ArmorFinder::initLightCoupleParam() {
    light_couple_param_.TWIN_ANGEL_MAX = 5.001;
    light_couple_param_.TWIN_LENGTH_RATIO_MAX = 2.0;
    light_couple_param_.TWIN_DISTANCE_N_MIN = 1.3;       // 1.7
    light_couple_param_.TWIN_DISTANCE_N_MAX = 3.8;       // 3.8
    light_couple_param_.TWIN_DISTANCE_T_MAX = 1.4;
    light_couple_param_.TWIN_AREA_MAX = 1.2;
    light_couple_param_.TWIN_CENTER_POSITION_DIFF_RATIO = 0.5;
}

void ArmorFinder::initArmorSeekingParam() {
    armor_seeking_param_.BORDER_IGNORE = 10;
    armor_seeking_param_.BOX_EXTRA = 10;
}


bool ArmorFinder::matchLightBlob(vector<LightBlob> &light_blobs, cv::Rect2d &armor_box) {
    if (light_blobs.size() < 2)
        return false;
    long light_index_left = -1;
    long light_index_right = -1;
    sort(light_blobs.begin(), light_blobs.end());
    //cout<<"light blobs vector size: "<<light_blobs.size()<<endl;

    for(long i = 0; i < light_blobs.size()-1; ++i)
    {
        long j = i + 1;
        if(!isCoupleLight(light_blobs.at(i), light_blobs.at(j)))
        {
            continue;
        }
        light_index_left = i;
        light_index_right = j;
    }
    if(light_index_left + light_index_right == -2)
    {
        return false;
    }

    Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(light_index_left)).rect.boundingRect();
    Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(light_index_right)).rect.boundingRect();
    double min_x, min_y, max_x, max_y;
    min_x = min(rect_left.x, rect_right.x) - armor_seeking_param_.BOX_EXTRA;
    max_x = max(rect_left.x+rect_left.width, rect_right.x + rect_right.width) + armor_seeking_param_.BOX_EXTRA;
    min_y = min(rect_left.y, rect_right.y) - armor_seeking_param_.BOX_EXTRA;
    max_y = max(rect_left.y+rect_left.height, rect_right.y+rect_right.height) + armor_seeking_param_.BOX_EXTRA;
    if (min_x < 0 || max_x > SRC_WIDTH || min_y < 0 || max_y > SRC_HEIGHT) {
        return false;
    }
    armor_box = Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
    return true;
}


bool ArmorFinder::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {

    Size2f size_i = light_blob_i.rect.size;
    Size2f size_j = light_blob_j.rect.size;
    float length_i = max(size_i.height, size_i.width);
    float length_j = max(size_j.height, size_j.width);
    if(length_i / length_j > light_couple_param_.TWIN_LENGTH_RATIO_MAX ||
       length_j/ length_i > light_couple_param_.TWIN_LENGTH_RATIO_MAX)
    {
        //cout<<"length similar fail."<<endl;
        return false;
    }

    Point2f center_i = light_blob_i.rect.center;
    Point2f center_j = light_blob_j.rect.center;
    if(abs(center_i.y - center_j.y) / max(length_i, length_j) > light_couple_param_.TWIN_CENTER_POSITION_DIFF_RATIO)
    {
        //cout<<"position y diff too much"<<endl;
        return false;
    }

    float angle_i = light_blob_i.rect.angle;
    float angle_j = light_blob_j.rect.angle;
    //cout<<"angles: "<<angle_i<<" "<<angle_j<<endl;
    if(abs(angle_i - angle_j) > light_couple_param_.TWIN_ANGEL_MAX)
    {
        //cout<<"angle similar fail"<<endl;
        return false;
    }
    ;

    return true;
}

