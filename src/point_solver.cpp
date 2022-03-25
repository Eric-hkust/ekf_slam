#include <cfloat>
#include <cmath>
#include <limits>
#include "../include/ekf_slam/point_solver.h"
#include "../include/ekf_slam/icp.h"

const double MAX_DIS_POINT = 0.2*0.2;
const double MAX_DIS_OBJECT = 0.1*0.1;
const double ZERO = 0.0001;
const double PI = 3.1415926535;

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
        temp_point.angle = 0;
        coord_points.push_back(temp_point);
    }
}

void PointSolver::coord_point_to_polar(const double & x_, const double & y_, const double & angle_, const std::vector<CoordPoint> & coord_points, std::vector<PolarPoint> & polar_points)
{
    PolarPoint temp_point;
    polar_points.clear();
    for(auto i:coord_points){
        temp_point.range = pow(pow(i.x-x_,2)+pow(i.y-y_,2),0.5);
        temp_point.angle = norm_angle(atan2(i.y-y_,i.x-x_)-angle_);
        polar_points.push_back(temp_point);
    }
}

size_t PointSolver::cluster_point(const std::vector<PolarPoint> & point_cloud, std::vector<std::vector<PolarPoint>> & points_cluster)
{
    points_cluster.clear();
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
                points_cluster.push_back(temp_point);
                temp_point.clear();
                temp_point.push_back(i);
            }
        }
    }
    if(temp_point.size()>0)
        points_cluster.push_back(temp_point);
    return points_cluster.size();    
}

size_t PointSolver::points_cluster_to_object(const double & x_, const double & y_, const double & angle_, const std::vector<std::vector<PolarPoint>> & points_cluster, std::vector<CoordPoint> & object)
{
    object.clear();
    std::vector<PolarPoint> object_polar; 
    for(auto i:points_cluster){
        if(i.size()>2){
            size_t count = 0;
            PolarPoint temp(0,0);
            for(auto j:i){
                temp.angle += j.angle;
                temp.range += j.range;
            }
            temp.range = temp.range / i.size();
            temp.angle = temp.angle / i.size();
            object_polar.push_back(temp);
        }      
    }
    polar_point_to_coord(x_,y_,angle_,object_polar,object);
    return object.size();
}

size_t PointSolver::points_cluster_to_column(const double & x_, const double & y_, const double & angle_, const double & r_, const std::vector<std::vector<PolarPoint>> & points_cluster, std::vector<CoordPoint> & colunm)
{
    std::vector<std::vector<CoordPoint>> points;
    std::vector<CoordPoint> temp;
    double x = x_;
    double y = y_;
    double angle = angle_;
    double r = r_;
    for(auto i:points_cluster){
        polar_point_to_coord(x,y,angle,i,temp);
        if(temp.size()>=3)
            points.push_back(temp);
    }
    for(auto i:points){
        double min_cost = DBL_MAX;
        CoordPoint column_best(0,0,0);
        for(auto j:i){
            double cost = 0;
            double angle = atan2(j.y-y, j.x-x);
            CoordPoint column_temp(j.x + r*cos(angle), j.y + r*sin(angle),0);
            for(auto k:i)
                cost += fabs(pow(pow(column_temp.x-k.x,2)+pow(column_temp.y-k.y,2),0.5)-r);
            if(cost<min_cost){
                column_best.x = column_temp.x;
                column_best.y = column_temp.y;
                min_cost = cost;
            }
        }
        colunm.push_back(column_best);
    }
    return colunm.size();
}

void PointSolver::estimate_object_pose(const double & x_, const double & y_, const double & angle_, const double & r_, std::vector<CoordPoint> & object, const bool flag)
{    
    object.clear();
    std::vector<std::vector<PolarPoint>> points_cluster;
    cluster_point(new_point_cloud, points_cluster);
    if(flag == true)
        points_cluster_to_column(x_,y_,angle_,r_,points_cluster,object);
    else
        points_cluster_to_object(x_,y_,angle_,points_cluster,object);
}

size_t PointSolver::estimate_obstacle(const double & x_, const double & y_, const double & angle_, const double & r_, std::vector<double> & obstacle, const bool flag)
{    
    obstacle.clear();
    if(new_point_cloud.size()==0 || old_point_cloud.size()==0)
        return 0;

    std::vector<std::vector<PolarPoint>> old_points_cluster,new_points_cluster;
    std::vector<CoordPoint> temp_new_object, temp_old_object;
    std::vector<CoordPoint> new_object, old_object;
    cluster_point(new_point_cloud,new_points_cluster);
    cluster_point(old_point_cloud,old_points_cluster);
    if(flag == true){
        points_cluster_to_column(x_,y_,angle_,r_,new_points_cluster,temp_new_object);
        points_cluster_to_column(x_,y_,angle_,r_,old_points_cluster,temp_old_object);
    }
    else{
        points_cluster_to_object(x_,y_,angle_,new_points_cluster,temp_new_object);
        points_cluster_to_object(x_,y_,angle_,old_points_cluster,temp_old_object);
    }
    for(auto i:temp_new_object){
        for(auto j:temp_old_object){
            if((pow(i.x-j.x,2)+pow(i.y-j.y,2))<0.01){
                new_object.push_back(i);
                old_object.push_back(j);
                break;
            }
        }
    }
    if(new_object.size()==0){
        std::cout<<"there is no obstacle!"<<std::endl;
        return 0;
    }
    else{
        for(auto i:new_object){
            obstacle.push_back(i.x);
            obstacle.push_back(i.y);
        }
        return new_object.size();
    }
    
}

void PointSolver::estimate_move(const double & angle_, CoordPoint & movement)
{
    if(new_point_cloud.size()==0 || old_point_cloud.size()==0)
        return;
    
    std::vector<std::vector<PolarPoint>> old_points_cluster,new_points_cluster;
    std::vector<CoordPoint> temp_new_object, temp_old_object;
    std::vector<CoordPoint> new_object, old_object;
    cluster_point(new_point_cloud,new_points_cluster);
    cluster_point(old_point_cloud,old_points_cluster);
    points_cluster_to_column(0,0,0,0.05,new_points_cluster,temp_new_object);
    points_cluster_to_column(0,0,0,0.05,old_points_cluster,temp_old_object);

    for(auto i:temp_new_object){
        for(auto j:temp_old_object){
            if((pow(i.x-j.x,2)+pow(i.y-j.y,2))<0.01){
                new_object.push_back(i);
                old_object.push_back(j);
                break;
            }
        }
    }
    if(new_object.size()<3){
        std::cout<<"warn: object number "<<new_object.size()<<std::endl;
        movement.x = 0;
        movement.y = 0;
        return;
    }

    Eigen::MatrixXd A(new_object.size(),3);
    Eigen::MatrixXd B(old_object.size(),3);
    for(size_t i=0;i<new_object.size();i++){
        A(i,0) = new_object[i].x;
        A(i,1) = new_object[i].y;
        A(i,2) = 0;
    }
    for(size_t i=0;i<old_object.size();i++){
        B(i,0) = old_object[i].x;
        B(i,1) = old_object[i].y;
        B(i,2) = 0;
    }
    ICP_OUT ipc_out = ICP(B,A,20,0.001);
    
    // movement.angle = atan2(ipc_out.trans(0,1),ipc_out.trans(0,0));
    // movement.x = - ipc_out.trans(0,3) * cos(angle_+movement.angle/2) + ipc_out.trans(1,3) * sin(angle_+movement.angle/2);
    // movement.y = - ipc_out.trans(0,3) * sin(angle_+movement.angle/2) - ipc_out.trans(1,3) * cos(angle_+movement.angle/2);
    movement.x = - ipc_out.trans(0,3) * cos(angle_);
    movement.y = - ipc_out.trans(0,3) * sin(angle_);
}

double PointSolver::norm_angle(const double & angle_)
{
     double angle = angle_;
     if(angle>2*PI){
          while(angle > 2*PI){
               angle -= PI*2;
          }
          return angle;
     }
     if(angle<0){
          while(angle < 0){
               angle += PI*2;
          }
          return angle;
     }
     return angle;
}
