#ifndef _CXZ_EKF_SLAM_SOLVER_H_
#define _CXZ_EKF_SLAM_SOLVER_H_

#include <Eigen/Core>
#include <Eigen/Dense>

class EkfSlamSolver
{
public:
    double pose[3];
    EkfSlamSolver();
    EkfSlamSolver(const double & x, const double & y, const double & angle);
    void predict(const double & v, const double & w, const double & dt);
private:
    Eigen::VectorXd pose_es;
    Eigen::MatrixXd cov;
    size_t matrix_size;
    Eigen::Matrix3d R;
    Eigen::Matrix3d Q;
};

#endif