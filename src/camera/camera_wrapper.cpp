//
// Created by zhikun on 18-11-7.
//

#include <camera/camera_wrapper.h>

using std::cout;
using std::endl;
using namespace cv;

CameraWrapper::CameraWrapper() {
    camera_cnts = 2;
    camera_status = -1;
    iplImage = nullptr;
    channel = 3;
}


bool CameraWrapper::init() {
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    int camera_enumerate_device_status = CameraEnumerateDevice(camera_enum_list, &camera_cnts);
    //cout<<"camera enumerate device status: "<<camera_enumerate_device_status<<endl;
    //cout<<"camera number: "<<camera_cnts<<endl;

    //没有连接设备
    if (camera_cnts == 0) {
        cout << "No device detected!" << endl;
        return false;
    } else if (camera_cnts == 1) {
        cout << "One camera device detected" << endl;
    } else {
        cout << "More than 1 cameras detected or some other error occurs." << endl;
        return false;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    camera_status = CameraInit(&camera_enum_list[0], -1, -1, &h_camera);
    //初始化失败
    if (camera_status != CAMERA_STATUS_SUCCESS) {
        cout << "Camera 0 initialization failed with code " << camera_status
             << ". See camera_status.h to find the code meaning." << endl;
        return false;
    }

    CameraGetFriendlyName(h_camera, camera_name);
    cout << "camera names: " << camera_name << endl;

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera, &tCapability);

    // set resolution to 320*240
    // CameraSetImageResolution(hCamera, &(tCapability.pImageSizeDesc[2]));

    rgb_buffer0 = (unsigned char *) malloc(tCapability.sResolutionRange.iHeightMax *
                                           tCapability.sResolutionRange.iWidthMax * 3);

    CameraSetAeState(h_camera, true);  //设置是否自动曝光

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(h_camera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         CameraGetFriendlyName    CameraSetFriendlyName 获取/设置相机名称（该名称可写入相机硬件）
    */
    double exposure_time0 = 0;
    CameraGetExposureTime(h_camera, &exposure_time0);
    cout << "exposure time " << exposure_time0 << endl;

    // 抗频闪
    CameraSetAntiFlick(h_camera, true);


    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);
        cout << "camera0 mono " << endl;
    } else {
        channel = 3;
        CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
        cout << "camera0 color mode" << endl;
    }

    return true;
}


bool CameraWrapper::read(cv::Mat &src0) {
    return readRaw(
            src0);             //suit for using bayer hacking in armor_finder to replace process, fast and it can filter red and blue.
    //return readProcessed(src0, src1);   // processed color image, but this runs slowly, about half fps of previous one.
    //return read_thread(src0, src1);     // read from camera using two threads, it seems of no use
}


bool CameraWrapper::readRaw(cv::Mat &src0) {
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 1000) == CAMERA_STATUS_SUCCESS) {
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }

        iplImage = cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, 1);

        cvSetData(iplImage, pby_buffer, frame_info.iWidth);  //此处只是设置指针，无图像块数据拷贝，不需担心转换效率

        src0 = cv::cvarrToMat(iplImage);

        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);

//        double exposure_time0 = 0, exposure_time1 = 0;
//        CameraGetExposureTime(h_camera, &exposure_time0);
//        CameraGetExposureTime(h_camera1, &exposure_time1);
//        cout<<"exposure time "<<exposure_time0<<" "<<exposure_time1<<endl;

        return true;
    } else {
        return false;
    }
}

bool CameraWrapper::readProcessed(cv::Mat &src0, cv::Mat &src1) {
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 1000) == CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(h_camera, pby_buffer, rgb_buffer0,
                           &frame_info);// this function is super slow, better not to use it.
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }

        iplImage = cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, channel);

        cvSetData(iplImage, rgb_buffer0, frame_info.iWidth * channel);  //此处只是设置指针，无图像块数据拷贝，不需担心转换效率

        src0 = cv::cvarrToMat(iplImage);

        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(h_camera, pby_buffer);
        return true;
    } else {
        return false;
    }
}


CameraWrapper::~CameraWrapper() {
    CameraUnInit(h_camera);
    //注意，先反初始化后再free
    free(rgb_buffer0);
}


void CameraWrapper::read_camera_thread0(cv::Mat &src) {
    if (CameraGetImageBuffer(h_camera, &frame_info, &pby_buffer, 1000) == CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(h_camera, pby_buffer, rgb_buffer0, &frame_info);
        iplImage = cvCreateImageHeader(cvSize(frame_info.iWidth, frame_info.iHeight), IPL_DEPTH_8U, channel);
        cvSetData(iplImage, rgb_buffer0, frame_info.iWidth * channel);
        src = cv::cvarrToMat(iplImage);
        CameraReleaseImageBuffer(h_camera, pby_buffer);
        read_state0 = true;
    } else {
        read_state0 = false;
    }

}

bool CameraWrapper::read_thread(cv::Mat &src0, cv::Mat &src1) {
    std::thread t1(&CameraWrapper::read_camera_thread0, this, std::ref(src0));
    t1.join();
    return read_state0;
}





