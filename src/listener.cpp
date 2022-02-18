#include <fstream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "../include/ekf_slam/point_solver.h"
#include "../include/ekf_slam/ekf_slam_solver.h"

const std::vector<double> CONST_OBSTACLE = {-2,-2,-2,-1,-2,0,-2,1,-2,2, -1,-2,-1,-1,-1,0,-1,1,-1,2, 0,-2,0,-1,0,1,0,2, 1,-2,1,-1,1,0,1,1,1,2, 2,-2,2,-1,2,0,2,1,2,2};

CoordPoint current_pose;
CoordPoint estimate_pose;
PointSolver point_solver;
EkfSlamSolver ekf_slam_solver(0,0,0,CONST_OBSTACLE);
EkfSlamSolver odom_estimator;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //get position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;
  //get 2d eular_angle
  double roll,pitch,yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);
  current_pose.angle = yaw;
  // ROS_INFO("Node listener heard Odom: x=%f, y=%f, eular_angle=%f", current_pose.x, current_pose.y, current_pose.angle);
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double acceleration[2];
  double angular_velocity;
  double eular_angle;
  
  // get 2d eular_angle
  double roll,pitch,yaw;
  tf2::Quaternion quat_tf;
  tf2::fromMsg(msg->orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll,pitch,yaw);
  eular_angle = yaw;
  // get acceleration
  acceleration[0] = msg->linear_acceleration.x;
  acceleration[1] = msg->linear_acceleration.y;
  //get angular_velocity
  angular_velocity = msg->angular_velocity.z;
  
  // ROS_INFO("Node listener heard Odom: eular_angle=%f, acceleration=(%f,%f), angular_velocity=%f", \
           eular_angle,acceleration[0], acceleration[1], angular_velocity);
}

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<PolarPoint> polar_points;
  std::vector<CoordPoint> coord_points;
  CoordPoint movement;
  double angle_increment;
  size_t scan_size;

  angle_increment = msg->angle_increment;
  scan_size = msg->ranges.size();
  PolarPoint new_point;
  for(size_t i=0; i<scan_size; i++){
    if(isfinite(msg->ranges[i])){
      new_point.range = msg->ranges[i];
      new_point.angle = i*angle_increment;
      polar_points.push_back(new_point);
    }
  }

  point_solver.update_point_cloud(polar_points);
  ekf_slam_solver.observe(point_solver);

  // point_solver.polar_point_to_coord(current_pose.x,current_pose.y,current_pose.angle,polar_points,coord_points);  
  // for(size_t i=0; i<coord_points.size(); i++){
  //   ROS_INFO("Node listener heard scan: point[%ld]=(%f,%f)",i,coord_points[i].x,coord_points[i].y);
  // }
}

void VelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v = msg->linear.x;
  double w = msg->angular.z;
  odom_estimator.predict(v,w,0.1);
  ekf_slam_solver.predict(v,w,0.1);
  estimate_pose.x = ekf_slam_solver.pose[0];
  estimate_pose.y = ekf_slam_solver.pose[1];
  estimate_pose.angle = ekf_slam_solver.pose[2];

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(ekf_slam_solver.pose[0], ekf_slam_solver.pose[1], 0.0));
  q.setRPY(0, 0, ekf_slam_solver.pose[2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "ekf_estimator"));

  transform.setOrigin(tf::Vector3(odom_estimator.pose[0], odom_estimator.pose[1], 0.0));
  q.setRPY(0, 0, odom_estimator.pose[2]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "odom_estimator"));

  // ROS_INFO("Node listener heard vel: vel=%f,w=%f",v,w);
  ROS_INFO("odom estimation:%f,%f,%f",odom_estimator.pose[0],odom_estimator.pose[1],odom_estimator.pose[2]);
  ROS_INFO("ekf estimation:%f,%f,%f",estimate_pose.x,estimate_pose.y,estimate_pose.angle);
  ROS_INFO("Node listener heard Odom: x=%f, y=%f, eular_angle=%f", current_pose.x, current_pose.y, current_pose.angle);
}

int main(int argc, char **argv)
{
  std::ofstream odom,ekf,real,error;
  odom.open("odom_path.txt");
  ekf.open("ekf_path.txt");
  real.open("real_path.txt");
  error.open("error.txt");

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, OdomCallback);
  ros::Subscriber sub2 = n.subscribe("/imu", 1000, ImuCallback);
  ros::Subscriber sub3 = n.subscribe("/scan", 1000, ScanCallback);
  ros::Subscriber sub4 = n.subscribe("/cmd_vel", 1000, VelCallback);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    odom<<odom_estimator.pose[0]<<" "<<odom_estimator.pose[1]<<std::endl;
    ekf<<ekf_slam_solver.pose[0]<<" "<<ekf_slam_solver.pose[1]<<std::endl;
    real<<current_pose.x<<" "<<current_pose.y<<std::endl;
    error<<pow(pow(odom_estimator.pose[0]-current_pose.x,2)+pow(odom_estimator.pose[1]-current_pose.y,2),0.5)<< " "<<pow(pow(ekf_slam_solver.pose[0]-current_pose.x,2)+pow(ekf_slam_solver.pose[1]-current_pose.y,2),0.5)<<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  odom.close();
  ekf.close();
  real.close();
  error.close();

  return 0;
}