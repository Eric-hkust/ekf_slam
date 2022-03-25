#ifndef _CXZ_POINT_SOLVER_H_
#define _CXZ_POINT_SOLVER_H_

#include <vector>
#include <math.h>
#include <iostream>

class PolarPoint{
public:
    double range;
    double angle;
    PolarPoint();
    PolarPoint(const double &range_, const double &angle_);
};

class CoordPoint{
public:
    double x;
    double y;
    double angle;
    CoordPoint();
    CoordPoint(const double & x_, const double & y_, const double & angle_);
};

class PointSolver
{
public:
    std::vector<PolarPoint> new_point_cloud, old_point_cloud;
    PointSolver();
    void update_point_cloud(const std::vector<PolarPoint> & input_new_point_cloud);
    void estimate_object_pose(const double & x_, const double & y_, const double & angle_, const double & r_, std::vector<CoordPoint> & object, const bool flag=true);
    size_t estimate_obstacle(const double & x_, const double & y_, const double & angle_, const double & r_, std::vector<double> & obstacle, const bool flag=true);
    void estimate_move(const double & angle_, CoordPoint & movement);
    void polar_point_to_coord(const double & x_, const double & y_, const double & angle_, const std::vector<PolarPoint> & polar_points, std::vector<CoordPoint> & coord_points);
    void coord_point_to_polar(const double & x_, const double & y_, const double & angle_, const std::vector<CoordPoint> & coord_points, std::vector<PolarPoint> & polar_points);
private:
    size_t cluster_point(const std::vector<PolarPoint> & point_cloud, std::vector<std::vector<PolarPoint>> & points_cluster);
    size_t points_cluster_to_object(const double & x_, const double & y_, const double & angle_, const std::vector<std::vector<PolarPoint>> & points_cluster, std::vector<CoordPoint> & object);
    size_t points_cluster_to_column(const double & x_, const double & y_, const double & angle_, const double & r_, const std::vector<std::vector<PolarPoint>> & points_cluster, std::vector<CoordPoint> & colunm);
    double norm_angle(const double & angle_);
};

#endif