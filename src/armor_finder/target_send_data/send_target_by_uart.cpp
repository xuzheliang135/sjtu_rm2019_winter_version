#include "armor_finder/armor_finder.h"
#include <math.h>

using namespace cv;
using namespace std;

void ArmorFinder::initUartParam() {

}

template<class T>
class DataStorage {
    int p_start = 0, p_end = 0, p_data = 0, maxLength;
    T *data;

    void keepOrder() {
        if (p_end >= maxLength)p_end = 0;
        if (p_data >= maxLength)p_data = 0;

        if (p_start == p_end)p_start++;
        if (p_start >= maxLength)p_start = 0;
    }


public:
    explicit DataStorage(int length) {
        maxLength = length + 1;
        data = new T[maxLength];
    }

    void append(T newData) {
        data[p_end++] = newData;
        keepOrder();
    }

    bool hasNext() {
        return p_data != p_end;
    }

    T next() {
        T next = data[p_data++];
        keepOrder();
        return next;
    }

    void rewind() {
        p_data = p_start;
    }

    T getMaxAbs() {
        T res = 0;
        for (int i = 0; i < maxLength; ++i) {
            res = res >= abs(data[i]) ? res : abs(data[i]);
        }
        return res;
    }
};

void showData(DataStorage<float> data, const String &title) {
    const int width = 800, height = 600;
    Mat pic = Mat(height, width, CV_8UC3, Scalar(255, 255, 255));
    float now, last = height / 2.0f;
    int index = 0;
    data.rewind();
//    while (data.hasNext()) {
//        now = -data.next() * height / 2 / 45 + height / 2.0f;
//        line(pic, Point2f(index++, last), Point2f(index, now), Scalar(0, 255, 0));
//        last = now;
//    }
    float max = 320;
    while (data.hasNext()) {
        now = -data.next() * height / 2 / max + height / 2.0f;
        line(pic, Point2f(index++, last), Point2f(index, now), Scalar(0, 255, 0));
        last = now;
    }
    imshow(title, pic);
    waitKey(1);
}

int focus_length = 280;
float pai = 3.14159;
bool ArmorFinder::sendTargetByUart(float x, float y, float z) {

//    static float sum_x=0,sum_y=0;
//    static DataStorage<float> data_x(600);
//    sum_x+=x;
//    sum_y+=y;
//    data_x.append(x);
//    showData(data_x, "x");
    x = atan(x / focus_length) * 180 / pai;
    y = atan(y / focus_length) * 180 / pai;
    uart_.sendTarget(x, y, z);
    return true;
}



