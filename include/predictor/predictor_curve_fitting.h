//
// Created by zhikun on 19-1-22.
//

#ifndef SJTU_RM2019_WINTER_VERSION_PREDICTOR_CURVE_FITTING_H
#define SJTU_RM2019_WINTER_VERSION_PREDICTOR_CURVE_FITTING_H


#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <time.h>
#include <iomanip>
#include <ctime>
#include <chrono>

struct ArmorPredictingParam
{
    float N;//数据点个数
    float CAMERA_FRAME_TIME; //每两帧图像的间隔时间
    float PREDICT_TIME; //预测时间
    float W_SIGMA;
};

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;
    }

    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }
    // 存盘和读盘：留空
    virtual bool read( std::istream& in ) {}
    virtual bool write( std::ostream& out ) const {}
};

//误差模型 模板参数:观测值维度,类型,链接定点类型
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    double _x; //x值,y值为_measurement
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
    //计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }
    virtual bool read(std::istream &in) {}
    virtual bool write(std::ostream &out) const {}
};

class CurveFitting
{
public:
    CurveFitting();
    ~CurveFitting(){}
    bool predictArmorPosition_xy(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position);
    bool predictArmorPosition_xz(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position);
    bool predictArmorPosition(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position);
    void initPredictingParam();

    double predict_xyzt(std::vector<cv::Point3d> &armor_history_position, int axis,
            std::vector<clock_t> time_serial, clock_t predict_time);

    void predictPositionOverTime(std::vector<cv::Point3d> &armor_history_position,
            std::vector<clock_t> time_serial, clock_t predict_time_interval,
            cv::Point3d &armor_predicted_position);

private:


    ArmorPredictingParam armor_predicting_param;

};

#endif //SJTU_RM2019_WINTER_VERSION_PREDICTOR_CURVE_FITTING_H
