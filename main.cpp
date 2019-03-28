//
// Created by zhikun on 18-11-10.
//
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"
#include "armor_finder/constant.h"
#include "camera/camera_wrapper.h"
#include "camera/video_wrapper.h"
#include "camera/wrapper_head.h"
#include "tools/calibrate_tool.h"
#include <unistd.h>

#include <ctime>

using namespace cv;
using std::cin;
using std::cout;
using std::endl;
using std::fstream;
using std::ios;
using std::string;


int main()
{
    int enemy_color = ENEMY_BLUE;
    int from_camera = 1;
    cout << "Input 1 for camera, 0 for video files" << endl;
    cin >> from_camera;

    while (true) {

        WrapperHead *video;

        if(from_camera)
            video = new CameraWrapper;
        else
            video = new VideoWrapper(
                    "/home/xuzheliang135/Downloads/video/video_color_0.avi");

        if(video->init())
        {
            cout<<"Video source initialization successfully."<<endl;
        } else{
            continue;
        }

        Mat src_left, src_right;
        Mat src_left_parallel, src_right_parallel;

        ArmorFinder armor_finder;
        armor_finder.setEnemyColor(enemy_color);

        for(int i = 0; i < 5; i++)
        {
            video->read(src_left); // to eliminate the initial noise images
            video->read(src_left_parallel);
        }

        cout<<"start working"<<endl;
//        time_t start = time(nullptr);
//        int cnt = 0;
        bool ok = true;
        while (ok)
        {

#pragma omp parallel sections
        {
#pragma omp section
                { ok = video->read(src_left); }
#pragma omp section
            {
                //armor_finder.showImage("raw", src_left_parallel, src_right_parallel);
                armor_finder.run(src_left_parallel);
            }
        }
#pragma omp barrier

#pragma omp parallel sections
        {
#pragma omp section
                { ok = video->read(src_left_parallel); }
#pragma omp section
            {
                //armor_finder.showImage("raw", src_left, src_right);
                armor_finder.run(src_left);
            }
        }
#pragma omp barrier

            waitKey(1);
        }
//        time_t end = time(nullptr);
//        cout<<(double)(end - start) << "s."<<endl;

        delete video;
        cout<<"Program fails. Restarting"<<endl;

    }
    return 0;
}
//int main(){
//    ArmorFinder armor_finder;
//    while(true){
//        for (int i = 0; i < 1; ++i) {
//            armor_finder.sendTargetByUart(
//                    static_cast<float>(0.5),
//                    static_cast<float>(0),
//                    static_cast<float>(300));
//            usleep(10000);
//        }
//        for (int i = 0; i < 1; ++i) {
//            armor_finder.sendTargetByUart(
//                    static_cast<float>(-0.5),
//                    static_cast<float>(0),
//                    static_cast<float>(300));
//            usleep(10000);
//        }
//    }
//
//}
