#include "../include/ekf_slam/ekf_slam_solver.h"
using namespace Eigen;
const double ZERO = 0.0001;

EkfSlamSolver::EkfSlamSolver()
{
    pose[0] = pose[1] = pose[2] = 0;
    pose_es = VectorXd::Zero(3);
    cov = MatrixXd::Zero(3,3);
    matrix_size = 3;
    R << 0,0,0,
         0,0,0,
         0,0,0;
    Q << 0,0,0,
         0,0,0,
         0,0,0;        
}

EkfSlamSolver::EkfSlamSolver(const double & x, const double & y, const double & angle)
{
    pose_es = VectorXd::Zero(3);
    pose[0] = x; pose[1]=y; pose[2]=angle;
    pose_es << x,y,angle;
    cov = MatrixXd::Zero(3,3);
    matrix_size = 3;
    R << 0,0,0,
         0,0,0,
         0,0,0;
    Q << 0,0,0,
         0,0,0,
         0,0,0; 
}

void EkfSlamSolver::predict(const double & v, const double & w, const double & dt)
{
    Vector3d temp;
    MatrixXd F(matrix_size,3);
    Matrix3d temp_;
    MatrixXd G(matrix_size,matrix_size);

    F.setZero();
    temp_.setZero();
    G.setZero();

    temp[0] = v*dt*cos(pose[2]+0.5*w*dt);
    temp[1] = v*dt*sin(pose[2]+0.5*w*dt);
    temp[2] = w*dt;
    temp_(0,2) = -v*dt*sin(pose[2]+0.5*w*dt);
    temp_(1,2) = v*dt*cos(pose[2]+0.5*w*dt);

    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;
    pose_es =  pose_es + F*temp;
    pose[0] = pose_es[0]; pose[1] = pose_es[1]; pose[2] = pose_es[2];
    G = MatrixXd::Identity(matrix_size,matrix_size) + F*temp_*(F.transpose());
    cov = G*cov*(G.transpose()) + F*R*(F.transpose());
}