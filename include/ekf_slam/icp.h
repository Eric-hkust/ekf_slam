#include "Eigen/Eigen"
#include <vector>

#ifndef ICP_H
#define ICP_H

#define N_pt 30    // # of points in the datasets
#define N_tests 100    // # of test iterations
#define noise_sigma 0.01    // standard deviation error to be added
#define translation 2.1     // max translation of the test set
#define rotation 2        // max rotation (radians) of the test set


typedef struct{
    Eigen::Matrix4d trans;
    std::vector<float> distances;
    int iter;
}  ICP_OUT;

typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

Eigen::Matrix4d BestFitTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

ICP_OUT ICP(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations=20, int tolerance = 0.001);

// throughout method
NEIGHBOR NearestNeighbor(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
float Dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb);
float PlaneDist(const Eigen::MatrixXd& T);
float PlaneRot(const Eigen::MatrixXd& T);

#endif
