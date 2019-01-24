#include "armor_finder/armor_finder.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

struct con {
    double x, y;                    //轮廓位置
    int order;                      //轮廓向量contours中的第几个

    bool operator<(con &m) {
        if (y > m.y) return false;
        else if (y == m.y) {
            if (x < m.x) return true;
            else return false;
        } else return true;
    }

} con[15];

struct result {
    double bi;
    int num;

    bool operator<(result &m) {
        if (bi < m.bi)return true;
        else return false;
    }
} result[15];

static Mat num[15];
static Mat sample;

int deal(Mat &src, int order);

double compare(Mat &src, Mat &sample);

void Threshold(Mat &src, Mat &sample, int m);

int ArmorFinder::recognize_digits(Mat image) {
    const Mat &srcImage = image;
    Mat dstImage, grayImage, Image;
    srcImage.copyTo(dstImage);
    cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
    threshold(grayImage, Image, 51, 255, CV_THRESH_BINARY);

    //定义轮廓和层次结构
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(Image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    int i = 0;
    Point2f pp[5][4];
    vector<vector<Point>>::iterator It;
    Rect rect[10];
    for (It = contours.begin(); It < contours.end(); It++) {                        //画出可包围数字的最小矩形
        Point2f vertex[4];
        rect[i] = boundingRect(*It);

        vertex[0] = rect[i].tl();                                                           //矩阵左上角的点
        vertex[1].x = (float) rect[i].tl().x, vertex[1].y = (float) rect[i].br().y;           //矩阵左下方的点
        vertex[2] = rect[i].br();                                                           //矩阵右下角的点
        vertex[3].x = (float) rect[i].br().x, vertex[3].y = (float) rect[i].tl().y;           //矩阵右上方的点

        for (int j = 0; j < 4; j++)
            line(dstImage, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255), 1);

        con[i].x = (vertex[0].x + vertex[1].x + vertex[2].x + vertex[3].x) / 4.0;                  //根据中心点判断图像的位置
        con[i].y = (vertex[0].y + vertex[1].y + vertex[2].y + vertex[3].y) / 4.0;
        con[i].order = i;
        if (rect[i].area() > 500) i++;

    }

    sort(con, con + i);

    for (int j = 0; j < i; j++) {
        int k = con[j].order;
        srcImage(rect[k]).copyTo(num[j]);
        cvtColor(num[j], num[j], COLOR_RGBA2GRAY);
        threshold(num[j], num[j], 48, 255, CV_THRESH_BINARY);

        int number = deal(num[j], j + 1);
        if (number != -1) return number;


    }
    return -1;


}

void Threshold(Mat &src, Mat &sample, int m) {
    if (m == 5 || m == 3 || m == 4) {
        bitwise_not(sample, sample);

    }
    cvtColor(sample, sample, COLOR_BGR2GRAY);
    threshold(sample, sample, 48, 255, CV_THRESH_BINARY_INV);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(sample, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    vector<vector<Point>>::iterator It;
    Rect rect;
    It = contours.begin();                      //画出可包围数字的最小矩形
    rect = boundingRect(*It);
    Mat num1;
    sample(rect).copyTo(num1);
    result[m].bi = compare(src, num1);
    result[m].num = m;


}

int deal(Mat &src, int order) {
    for (int i = 0; i < 10; i++) {
        sample = imread("./template_num/" + to_string(i) + ".jpg");
        Threshold(src, sample, i);
    }

    sort(result, result + 10);

    if (result[9].bi > 0.66) {
        return result[9].num;
    } else return -1;

}

double compare(Mat &src, Mat &sample) {

    double same = 0.0, difPoint = 0.0;
    Mat now;
    resize(sample, now, src.size());

    int row = now.rows;
    int col = now.cols * now.channels();
    for (int i = 0; i < 1; i++) {
        uchar *data1 = src.ptr<uchar>(i);
        uchar *data2 = now.ptr<uchar>(i);
        for (int j = 0; j < row * col; j++) {
            int a = data1[j];
            int b = data2[j];
            if (a == b)same++;
            else difPoint++;
        }
    }
    return same / (same + difPoint);
}

