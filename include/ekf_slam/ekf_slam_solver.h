#ifndef _CXZ_EKF_SLAM_SOLVER_H_
#define _CXZ_EKF_SLAM_SOLVER_H_
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include "point_solver.h"

class EkfSlamSolver
{
public:
    double pose[3];
    EkfSlamSolver();
    EkfSlamSolver(const double & x, const double & y, const double & angle, const std::vector<double> & obstacle);
    void predict(const double & v, const double & w, const double & dt);
    void observe(PointSolver & point_solver);
private:
    Eigen::VectorXd pose_es;
    Eigen::MatrixXd cov;
    size_t matrix_size;
    size_t object_size;
    Eigen::Matrix3d R;
    Eigen::Matrix2d Q;
    void observe_one(const double & r, const double & angle, const size_t & match_index);
    bool in_previous_observation(const double &x, const double &y, size_t & match_index);
    double norm_angle(const double & angle_);
};

#endif