#include "armor_finder/armor_finder.h"

using namespace cv;


Mat ArmorFinder::getNumberPic(Mat &src, const Rect &rect) {
    return src(rect);
}