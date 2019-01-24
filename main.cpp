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
                    "/home/zhikun/Videos/video_color_0.avi",
                    "/home/zhikun/Videos/video_color_1.avi"
                    );

        if(video->init())
        {
            cout<<"Video source initialization successfully."<<endl;
        } else{
            continue;
        }

        Mat src_left, src_right;

        ArmorFinder armor_finder;
        armor_finder.setEnemyColor(enemy_color);

        for(int i = 0; i < 10; i++)
        {
            video->read(src_left, src_right); // to eliminate the initial noise images
        }
        cout<<"start working"<<endl;
        clock_t last_time = clock();
        while (video->read(src_left, src_right))
        {
            cout<<(clock() - last_time) * 1000.0 / CLOCKS_PER_SEC<<endl;
            last_time = clock();
            armor_finder.showTwoImages("raw", src_left, src_right);
//            //armor_finder.run(src_left, src_right);
//            if(tool_for_calibrate(src_left, src_right)){
//                waitKey(0);
//                return 0;
//            }
//            waitKey(500);
            waitKey(1);

        }

        delete video;
        cout<<"Program fails. Restarting"<<endl;

    }
    return 0;
}
