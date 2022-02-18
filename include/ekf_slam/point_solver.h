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
    void polar_point_to_coord(const double & x_, const double & y_, const double & angle_, const std::vector<PolarPoint> & polar_points, std::vector<CoordPoint> & coord_points);
    void update_point_cloud(const std::vector<PolarPoint> & input_new_point_cloud);
    void estimate_move(CoordPoint & movement);
    size_t cluster_point_to_object(std::vector<PolarPoint> & object);
    size_t match_object_object(const std::vector<PolarPoint> & object_1, const std::vector<PolarPoint> & object_2, std::vector<size_t> & index_match);
};

#endif