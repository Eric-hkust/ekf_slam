#include "../include/ekf_slam/point_solver.h"

const double MAX_DIS_POINT = 0.2*0.2;
const double MAX_DIS_OBJECT = 0.1*0.1;

PolarPoint::PolarPoint()
{
    range = 0;
    angle = 0;
}

PolarPoint::PolarPoint(const double &range_, const double &angle_)
{
    range = range_;
    angle = angle_;
}

CoordPoint::CoordPoint()
{
    x = 0;
    y = 0;
    angle = 0;
}

CoordPoint::CoordPoint(const double & x_, const double & y_, const double & angle_)
{
    x = x_;
    y = y_;
    angle = angle_;    
}

PointSolver::PointSolver()
{
    old_point_cloud.clear();
    new_point_cloud.clear();
}

void PointSolver::update_point_cloud(const std::vector<PolarPoint> & input_new_point_cloud)
{
    old_point_cloud.clear();
    for (auto i:new_point_cloud){
        old_point_cloud.push_back(i);
    }
    new_point_cloud.clear();
    for (auto i:input_new_point_cloud){
        new_point_cloud.push_back(i);
    }
}

void PointSolver::polar_point_to_coord(const double & x_, const double & y_, const double & angle_, const std::vector<PolarPoint> & polar_points, std::vector<CoordPoint> & coord_points)
{
    CoordPoint temp_point;
    coord_points.clear();
    for(auto i:polar_points){
        temp_point.x = x_ + i.range * cos(i.angle + angle_);
        temp_point.y = y_ + i.range * sin(i.angle + angle_);
        temp_point.angle = i.angle + angle_;
        coord_points.push_back(temp_point);
    }
}

size_t PointSolver::cluster_point_to_object(const std::vector<PolarPoint> & point_cloud, std::vector<PolarPoint> & object)
{
    object.clear();
    std::vector<PolarPoint> temp_point;
    PolarPoint temp;
    for(auto i:point_cloud){
        if(temp_point.size() == 0){
            temp_point.push_back(i);
        }
        else{
            PolarPoint last_point = temp_point.back();
            double distance = last_point.range*last_point.range + i.range*i.range - 2*last_point.range*i.range*cos(last_point.angle-i.angle);
            if(distance < MAX_DIS_POINT)
                temp_point.push_back(i);
            else{
                if(temp_point.size()>2){    
                    temp.angle = 0;
                    temp.range = 0;
                    for(auto j:temp_point){
                        temp.range = temp.range + j.range;
                        temp.angle = temp.angle + j.angle;
                    }
                    temp.range = temp.range / temp_point.size();
                    temp.angle = temp.angle / temp_point.size();
                    object.push_back(temp);
                }
                temp_point.clear();
                temp_point.push_back(i);
            }
        }
    }
    if(temp_point.size()>2){    
        temp.angle = 0;
        temp.range = 0;
        for(auto j:temp_point){
            temp.range = temp.range + j.range;
            temp.angle = temp.angle + j.angle;
        }
        temp.range = temp.range / temp_point.size();
        temp.angle = temp.angle / temp_point.size();
        object.push_back(temp);
    }

    return object.size();
}

size_t PointSolver::match_object_object(const std::vector<PolarPoint> & object_1, const std::vector<PolarPoint> & object_2, std::vector<size_t> & index_match)
{
    for(size_t i=0; i<object_1.size(); i++){
        for(size_t j=0; j<object_2.size(); j++){
            double distance = object_1[i].range*object_1[i].range + object_2[j].range*object_2[j].range - 2*object_1[i].range*object_2[j].range*cos(object_1[i].angle - object_2[j].angle);
            if(distance < MAX_DIS_OBJECT){
                index_match.push_back(i);
                index_match.push_back(j);
                break;
            }
        }
    }
    return index_match.size()/2;
}

void PointSolver::estimate_move(CoordPoint & movement)
{
    // ROS_INFO("Node listener heard scan: new point number = %ld, old point number = %ld", new_point_cloud.size(), old_point_cloud.size());
    std::vector<PolarPoint> objects1,objects2;
    std::vector<PolarPoint> new_objects_polar,old_objects_polar;
    std::vector<CoordPoint> new_objects_coord,old_objects_coord;
    std::vector<size_t> index_match;

    size_t object_number1 = cluster_point_to_object(new_point_cloud, objects1);
    size_t object_number2 = cluster_point_to_object(old_point_cloud,objects2);
    size_t match_number = match_object_object(objects1,objects2,index_match);
    // ROS_INFO("Node listener heard scan: new object's number = %ld, old object's number = %ld, match number = %ld", object_number1,object_number2,match_number);

    movement.x = 0;
    movement.y = 0;
    movement.angle = 0;
    for(size_t i=0; i<match_number; i++){
        new_objects_polar.push_back(objects1[index_match[2*i]]);
        old_objects_polar.push_back(objects2[index_match[2*i+1]]);
    }
    polar_point_to_coord(0,0,0,new_objects_polar,new_objects_coord);
    polar_point_to_coord(0,0,0,old_objects_polar,old_objects_coord);
}