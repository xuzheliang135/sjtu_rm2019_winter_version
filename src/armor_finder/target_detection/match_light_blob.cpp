
#include <armor_finder/armor_finder.h>

#include "armor_finder/armor_finder.h"

using namespace cv;
using namespace std;


void ArmorFinder::initLightCoupleParam() {
    light_couple_param_.TWIN_ANGEL_MAX = 10;
    light_couple_param_.TWIN_LENGTH_RATIO_MAX = 4.0;
    light_couple_param_.TWIN_DISTANCE_N_MIN = 1.3;       // 1.7
    light_couple_param_.TWIN_DISTANCE_N_MAX = 3.8;       // 3.8
    light_couple_param_.TWIN_DISTANCE_T_MAX = 1.4;
    light_couple_param_.TWIN_AREA_RATIO_MAX = 3;
    light_couple_param_.TWIN_CENTER_POSITION_DIFF = 100;
}

void ArmorFinder::initArmorSeekingParam() {
    armor_seeking_param_.BORDER_IGNORE = 10;
    armor_seeking_param_.BOX_EXTRA = 5;
}

bool ArmorFinder::matchLightBlobVector(std::vector<LightBlob> &light_blobs, vector<cv::Rect2d> &armor_box) {
    if (light_blobs.size() < 2)
        return false;
    long light_index_left = -1;
    long light_index_right = -1;

    sort(light_blobs.begin(), light_blobs.end(),
         [](LightBlob a, LightBlob b) -> bool { return a.rect.center.y > b.rect.center.y; });
    //cout<<"light blobs vector size: "<<light_blobs.size()<<endl;

    for (long i = 0; i < light_blobs.size() - 1; ++i) {
        for (long j = i + 1; j < light_blobs.size() - 1; ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j))) {
                continue;
            }
            light_index_left = i;
            light_index_right = j;
            Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(light_index_left)).rect.boundingRect();
            Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(light_index_right)).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = min(rect_left.x, rect_right.x) - armor_seeking_param_.BOX_EXTRA;
            max_x = max(rect_left.x + rect_left.width, rect_right.x + rect_right.width) +
                    armor_seeking_param_.BOX_EXTRA;
            min_y = min(rect_left.y, rect_right.y) - armor_seeking_param_.BOX_EXTRA;
            max_y = max(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
                    armor_seeking_param_.BOX_EXTRA;
            if (min_x < 0 || max_x > SRC_WIDTH || min_y < 0 || max_y > SRC_HEIGHT) {
                continue;
            }
            armor_box.emplace_back(Rect2d(min_x, min_y, max_x - min_x, max_y - min_y));
        }

    }
    if (light_index_left + light_index_right == -2) {
        return false;
    }


    return true;
}

bool ArmorFinder::matchLightBlob(vector<LightBlob> &light_blobs, cv::Rect2d &armor_box) {
    if (light_blobs.size() < 2)
        return false;
    long light_index_left = -1;
    long light_index_right = -1;

    sort(light_blobs.begin(), light_blobs.end(),
         [](LightBlob a, LightBlob b) -> bool { return a.rect.center.y > b.rect.center.y; });
    //cout<<"light blobs vector size: "<<light_blobs.size()<<endl;

    for (long i = 0; i < light_blobs.size() - 1; ++i) {
        for (long j = i + 1; j < light_blobs.size(); ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j))) {
                continue;
            }
            light_index_left = i;
            light_index_right = j;
            break;
        }
        if (light_index_left + light_index_right != -2) {
            break;
        }
    }
    if (light_index_left + light_index_right == -2) {
        return false;
    }

    Rect2d rect_left = light_blobs.at(light_index_left).rect.boundingRect();
    Rect2d rect_right = light_blobs.at(light_index_right).rect.boundingRect();
    double min_x, min_y, max_x, max_y;
    min_x = min(rect_left.x, rect_right.x) - armor_seeking_param_.BOX_EXTRA;
    max_x = max(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + armor_seeking_param_.BOX_EXTRA;
    min_y = min(rect_left.y, rect_right.y) - armor_seeking_param_.BOX_EXTRA;
    max_y = max(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + armor_seeking_param_.BOX_EXTRA;
    if (min_x < 0 || max_x > SRC_WIDTH || min_y < 0 || max_y > SRC_HEIGHT) {
        return false;
    }
    armor_box = Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
    return true;
}

bool tmpCoupleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    Point2f side = light_blob_i.rect.center - light_blob_j.rect.center;
    Point2f rect;
    if (light_blob_i.rect.size.width >= light_blob_i.rect.size.height)
        rect = Point2f(static_cast<float>(10 * cos(light_blob_i.rect.angle * 3.1415926 / 180)),
                       static_cast<float>(10 * sin(light_blob_i.rect.angle * 3.1415926 / 180)));
    else
        rect = Point2f(static_cast<float>(10 * cos((light_blob_i.rect.angle + 90) * 3.1415926 / 180)),
                       static_cast<float>(10 * sin((light_blob_i.rect.angle + 90) * 3.1415926 / 180)));
    //cout << abs(side.dot(rect) * side.dot(rect) / (side.dot(side) * rect.dot(rect))) << endl;
    return abs(side.dot(rect) * side.dot(rect) / (side.dot(side) * rect.dot(rect))) < 0.02;
}
bool ArmorFinder::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {

    Point2f center_i = light_blob_i.rect.center;
    Point2f center_j = light_blob_j.rect.center;
    if (abs(center_i.y - center_j.y) > light_couple_param_.TWIN_CENTER_POSITION_DIFF) {
        cout<<"position y diff too much: "<<center_i<<" "<<center_j<< endl;
        return false;
    }


    Size2f size_i = light_blob_i.rect.size;
    Size2f size_j = light_blob_j.rect.size;
    float length_i = max(size_i.height, size_i.width);
    float length_j = max(size_j.height, size_j.width);
//    if (length_i / length_j > light_couple_param_.TWIN_LENGTH_RATIO_MAX ||
//        length_j / length_i > light_couple_param_.TWIN_LENGTH_RATIO_MAX) {
//        cout<<"length similar fail."<<endl;
//        return false;
//    }


    Point2f side = center_i - center_j;
    if (length_i * length_j / side.dot(side) > 9 || length_i * length_j / side.dot(side) < (1.0f / 9))return false;

    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;

//    if (abs(angle_i - angle_j) > light_couple_param_.TWIN_ANGEL_MAX) {
//        cout<<"angle similar fail "<<angle_i<<" "<<angle_j<<endl;
//        return false;
//    };

    double area_i = light_blob_i.rect.boundingRect2f().area();
    double area_j = light_blob_j.rect.boundingRect2f().area();
    if(area_i / area_j > light_couple_param_.TWIN_AREA_RATIO_MAX || area_j / area_i > light_couple_param_.TWIN_AREA_RATIO_MAX)
    {
        cout<<"area similar fail "<<area_i<<" "<<area_j<<" "<<center_i<<" "<<center_j<<endl;
        return false;
    }


//    return tmpCoupleJudge(light_blob_i, light_blob_j);
    return true;
}

