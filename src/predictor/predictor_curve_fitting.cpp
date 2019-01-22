
#include <predictor/predictor_curve_fitting.h>

#include "predictor/predictor_curve_fitting.h"

CurveFitting::CurveFitting()
{
    initPredictingParam();
}

void CurveFitting::initPredictingParam()
{
    armor_predicting_param.N = 30;
    armor_predicting_param.CAMERA_FRAME_TIME = 0.04;
    armor_predicting_param.PREDICT_TIME = 0.5;
    armor_predicting_param.W_SIGMA = 1.0;
}

bool CurveFitting::predictArmorPosition_xy(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position)
{
//构件图优化,先设定g2o
    //矩阵块:每个误差项优化变量维度为3,误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block; // 每个误差项优化变量维度为3，误差值维度为1
    //线性方程求解器:稠密的增量方程
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    //g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 往图中增加顶点x-y
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i = 0; i < armor_predicting_param.N; i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(armor_history_position[i].x);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(armor_history_position[i].y);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (armor_predicting_param.W_SIGMA * armor_predicting_param.W_SIGMA));
        optimizer.addEdge(edge);
    }

    //执行优化x-y
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(1);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    // 输出优化值x-y
    Eigen::Vector3d abc_estimate = v->estimate();
//    std::cout << "estimated model x-y: " << abc_estimate.transpose() << std::endl;
    float speed_x = (armor_history_position[18].x - armor_history_position[0].x) / (armor_predicting_param.CAMERA_FRAME_TIME * (armor_predicting_param.N - 2));
    armor_predicted_position.x = armor_history_position[19].x + speed_x * armor_predicting_param.PREDICT_TIME;
    armor_predicted_position.y = abc_estimate(0, 0) * pow(armor_predicted_position.x, 2) + abc_estimate(1, 0) * armor_predicted_position.x + abc_estimate(2, 0);

    return true;
}

bool CurveFitting::predictArmorPosition_xz(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position)
{
    //构件图优化,先设定g2o
    //矩阵块:每个误差项优化变量维度为3,误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block; // 每个误差项优化变量维度为3，误差值维度为1
    //线性方程求解器:稠密的增量方程
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    //g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 往图中增加顶点x-z
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i = 0; i < armor_predicting_param.N; i++)
    {
        CurveFittingEdge *edge = new CurveFittingEdge(armor_history_position[i].x);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(armor_history_position[i].z);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (armor_predicting_param.W_SIGMA * armor_predicting_param.W_SIGMA));
        optimizer.addEdge(edge);
    }

    //执行优化x-z

    optimizer.initializeOptimization();
    optimizer.optimize(1);

    // 输出优化值x-z
    Eigen::Vector3d abc_estimate = v->estimate();
    std::cout << "estimated model x-z: " << abc_estimate.transpose() << std::endl;
    float speed_x = (armor_history_position[18].x - armor_history_position[0].x) / (armor_predicting_param.CAMERA_FRAME_TIME * (armor_predicting_param.N - 2));
    armor_predicted_position.x = armor_history_position[19].x + speed_x * armor_predicting_param.PREDICT_TIME;
    armor_predicted_position.z = abc_estimate(0, 0) * pow(armor_predicted_position.x, 2) + abc_estimate(1, 0) * armor_predicted_position.x + abc_estimate(2, 0);

    return true;
}

bool CurveFitting::predictArmorPosition(std::vector<cv::Point3d> &armor_history_position, cv::Point3d &armor_predicted_position)
{
    if(predictArmorPosition_xy(armor_history_position,armor_predicted_position)&&predictArmorPosition_xz(armor_history_position,armor_predicted_position))
    {
        return true;
    }

}

void CurveFitting::predictPositionOverTime(std::vector<cv::Point3d> &armor_history_position,
                                           std::vector<clock_t> time_serial,
                                           clock_t predict_time,
                                           cv::Point3d &armor_predicted_position) {
    armor_predicted_position.x = predict_xyzt(armor_history_position, 0, time_serial, predict_time);
    armor_predicted_position.y = predict_xyzt(armor_history_position, 1, time_serial, predict_time);
    armor_predicted_position.z = predict_xyzt(armor_history_position, 2, time_serial, predict_time);

}

double CurveFitting::predict_xyzt(std::vector<cv::Point3d> &armor_history_position, int axis,
                                  std::vector<clock_t> time_serial, clock_t predict_time) {
    //构件图优化,先设定g2o
    //矩阵块:每个误差项优化变量维度为3,误差值维度为1
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block; // 每个误差项优化变量维度为3，误差值维度为1
    //线性方程求解器:稠密的增量方程
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
    Block *solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    //g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // 往图中增加顶点
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(0, 0, 0));
    v->setId(0);
    optimizer.addVertex(v);
    clock_t data_clock;
    double data_curve_point;
    for (int i = 0; i < armor_history_position.size(); i+=5)
    {
        data_clock = time_serial[i];
        CurveFittingEdge *edge = new CurveFittingEdge(data_clock);
        edge->setId(i);
        edge->setVertex(0, v);
        switch (axis){
            case 0:data_curve_point = armor_history_position[i].x; break;
            case 1:data_curve_point = armor_history_position[i].y; break;
            case 2:data_curve_point = armor_history_position[i].z; break;
            default: std::cout<<"axis is not set correctly"<<std::endl;
        }
        edge->setMeasurement(data_curve_point);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (armor_predicting_param.W_SIGMA * armor_predicting_param.W_SIGMA));
        optimizer.addEdge(edge);
    }

    //执行优化

    optimizer.initializeOptimization();
    optimizer.optimize(1);

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
//    std::cout << "estimated model t- x/y/z: " << abc_estimate.transpose() << std::endl;
    double predicted_point = abc_estimate(0,0)* pow(predict_time, 2) + abc_estimate(1,0) * predict_time + abc_estimate(2,0);

    return predicted_point;
}
