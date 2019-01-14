#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <time.h>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "armor_finder/armor_finder.h"

#define CAMERA_FRAME_TIME 0.04 //每两帧图像的间隔时间
#define PREDICT_TIME 0.5       //预测时间

using namespace std;

#define ParaBuffer(Buffer, Row, Col) (*(Buffer + (Row) * (SizeSrc + 1) + (Col)))

static int ParalimitRow(double *Para, int SizeSrc, int Row)
{
    int i;
    double Max, Min, Temp;
    for (Max = abs(ParaBuffer(Para, Row, 0)), Min = Max, i = SizeSrc; i; i--)
    {
        Temp = abs(ParaBuffer(Para, Row, i));
        if (Max < Temp)
            Max = Temp;
        if (Min > Temp)
            Min = Temp;
    }
    Max = (Max + Min) * 0.000005;
    for (i = SizeSrc; i >= 0; i--)
        ParaBuffer(Para, Row, i) /= Max;
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int Paralimit(double *Para, int SizeSrc)
{
    int i;
    for (i = 0; i < SizeSrc; i++)
        if (ParalimitRow(Para, SizeSrc, i))
            return -1;
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int ParaPreDealA(double *Para, int SizeSrc, int Size)
{
    int i, j;
    for (Size -= 1, i = 0; i < Size; i++)
    {
        for (j = 0; j < Size; j++)
            ParaBuffer(Para, i, j) = ParaBuffer(Para, i, j) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, j) * ParaBuffer(Para, i, Size);
        ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, Size, Size) - ParaBuffer(Para, Size, SizeSrc) * ParaBuffer(Para, i, Size);
        ParaBuffer(Para, i, Size) = 0;
        ParalimitRow(Para, SizeSrc, i);
    }
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int ParaDealA(double *Para, int SizeSrc)
{
    int i;
    for (i = SizeSrc; i; i--)
        if (ParaPreDealA(Para, SizeSrc, i))
            return -1;
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int ParaPreDealB(double *Para, int SizeSrc, int OffSet)
{
    int i, j;
    for (i = OffSet + 1; i < SizeSrc; i++)
    {
        for (j = OffSet + 1; j <= i; j++)
            ParaBuffer(Para, i, j) *= ParaBuffer(Para, OffSet, OffSet);
        ParaBuffer(Para, i, SizeSrc) = ParaBuffer(Para, i, SizeSrc) * ParaBuffer(Para, OffSet, OffSet) - ParaBuffer(Para, i, OffSet) * ParaBuffer(Para, OffSet, SizeSrc);
        ParaBuffer(Para, i, OffSet) = 0;
        ParalimitRow(Para, SizeSrc, i);
    }
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int ParaDealB(double *Para, int SizeSrc)
{
    int i;
    for (i = 0; i < SizeSrc; i++)
        if (ParaPreDealB(Para, SizeSrc, i))
            return -1;
    for (i = 0; i < SizeSrc; i++)
    {
        if (ParaBuffer(Para, i, i))
        {
            ParaBuffer(Para, i, SizeSrc) /= ParaBuffer(Para, i, i);
            ParaBuffer(Para, i, i) = 1.0;
        }
    }
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int ParaDeal(double *Para, int SizeSrc)
{
    //PrintPara(Para, SizeSrc);
    Paralimit(Para, SizeSrc);
    //PrintPara(Para, SizeSrc);
    if (ParaDealA(Para, SizeSrc))
        return -1;
    //PrintPara(Para, SizeSrc);
    if (ParaDealB(Para, SizeSrc))
        return -1;
    return 0;
}
/***********************************************************************************
 ***********************************************************************************/
static int GetParaBuffer(double *Para, const double *X, const double *Y, int Amount, int SizeSrc)
{
    int i, j;
    for (i = 0; i < SizeSrc; i++)
        for (ParaBuffer(Para, 0, i) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, 0, i) += pow(*(X + j), 2 * (SizeSrc - 1) - i);
    for (i = 1; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc - 1) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc - 1) += pow(*(X + j), SizeSrc - 1 - i);
    for (i = 0; i < SizeSrc; i++)
        for (ParaBuffer(Para, i, SizeSrc) = 0, j = 0; j < Amount; j++)
            ParaBuffer(Para, i, SizeSrc) += (*(Y + j)) * pow(*(X + j), SizeSrc - 1 - i);
    for (i = 1; i < SizeSrc; i++)
        for (j = 0; j < SizeSrc - 1; j++)
            ParaBuffer(Para, i, j) = ParaBuffer(Para, i - 1, j + 1);
    return 0;
}
//***********************************************************************************
//***********************************************************************************
int Cal(const double *BufferX, const double *BufferY, int Amount, int SizeSrc, double *ParaResK)
{
    auto *ParaK = (double *)malloc(SizeSrc * (SizeSrc + 1) * sizeof(double));
    GetParaBuffer(ParaK, BufferX, BufferY, Amount, SizeSrc);
    ParaDeal(ParaK, SizeSrc);
    for (Amount = 0; Amount < SizeSrc; Amount++, ParaResK++)
        *ParaResK = ParaBuffer(ParaK, Amount, SizeSrc);
    free(ParaK);
    return 0;
}


void ArmorFinder::initArmorPredictParam(){
    armor_predict_param_.ARMOR_POSITION_HISTORY_MAX_LENGTH = 100;
}

void ArmorFinder::manageHistorySpacePosition(const cv::Point3d &space_position) {
    armor_history_positions_.push_back(space_position);
    if(armor_history_positions_.size() > armor_predict_param_.ARMOR_POSITION_HISTORY_MAX_LENGTH)
    {
        armor_history_positions_.erase(armor_history_positions_.begin());
    }
}


bool ArmorFinder::predictArmorPosition(cv::Point3d &armor_position,
                                        cv::Point3d &armor_predicted_position) //预测0.5s之后的空间点
{
    manageHistorySpacePosition(armor_position);
    double finalX, finalY, finalZ, speed, tempX[1024], tempY[1024], tempZ[1024], ParaK_XY[6], ParaK_XZ[6]; //5次拟合, 一共6个系数(包含常数项)
    for(int i =0;i<19;i++)
    {
        tempX[i]=armor_history_positions_[i].x;
        tempY[i]=armor_history_positions_[i].y;
        tempZ[i]=armor_history_positions_[i].z;
    }
    Cal((const double *)tempX, (const double *)tempY, 20, sizeof(ParaK_XY) / sizeof(double), (double *)ParaK_XY);

    Cal((const double *)tempX, (const double *)tempZ, 20, sizeof(ParaK_XZ) / sizeof(double), (double *)ParaK_XZ);

    speed = (tempX[19] - tempX[10]) / (CAMERA_FRAME_TIME * 18);
    finalX = speed * PREDICT_TIME;
    finalY = ParaK_XY[0] + ParaK_XY[1] * finalX + ParaK_XY[2] * pow(finalX, 2) + ParaK_XY[3] * pow(finalX, 3) + ParaK_XY[4] * pow(finalX, 4) + ParaK_XY[5] * pow(finalX, 5);
    finalZ = ParaK_XZ[0] + ParaK_XZ[1] * finalX + ParaK_XZ[2] * pow(finalX, 2) + ParaK_XZ[3] * pow(finalX, 3) + ParaK_XZ[4] * pow(finalX, 4) + ParaK_XZ[5] * pow(finalX, 5);
    armor_predicted_position.x = finalX;
    armor_predicted_position.y = finalY;
    armor_predicted_position.z = finalZ;
    return true;
}
