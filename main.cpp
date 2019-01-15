//
// Created by zhikun on 18-11-10.
//
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"
#include "camera/camera_wrapper.h"
#include "camera/video_wrapper.h"
#include "camera/wrapper_head.h"

#include <time.h>

using namespace cv;
using std::cin;
using std::cout;
using std::endl;
using std::fstream;
using std::ios;
using std::string;


int main()
{
    int key;
    while(true)
    {
        int from_camera = 1;
        cout<<"Input 1 for camera, 0 for video files"<<endl;
        cin>>from_camera;
        //from_camera = waitKey(1000);

        WrapperHead *video;

        if(from_camera)
            video = new CameraWrapper;
        else
            video = new VideoWrapper(
                    "/home/zhikun/Videos/video_horizontal_move_0.avi",
                    "/home/zhikun/Videos/video_horizontal_move_1.avi"
                    );


        if(video->init())
        {
            cout<<"Video source initialization successfully."<<endl;
        } else{
            continue;
        }


        Mat src_left, src_right;

        ArmorFinder armor_finder;

        cout<<"start working"<<endl;

        //for(int i = 0; i < 10; i++) video->read(src_left, src_right);

        while (video->read(src_left, src_right))
        {
            imshow("left", src_left);
            imshow("right", src_right);

//            if(!from_camera)
//            {
//                cvtColor(src_left, src_left, COLOR_BGR2GRAY);
//                cvtColor(src_right, src_right, COLOR_BGR2GRAY);
//            }
            armor_finder.run(src_left, src_right);
            waitKey(1);

        }
        delete video;
        cout<<"Program fails. Restarting"<<endl;

    }
    return 0;
}