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
CoordPoint estimate_icp;
CoordPoint estimate_ekf;
CoordPoint estimate_predict;
PointSolver point_solver;
EkfSlamSolver ekf_slam_solver(0,0,0,CONST_OBSTACLE);
EkfSlamSolver odom_predict_solver;
double error;

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

  // get icp angle
  estimate_icp.angle = eular_angle;
  // // get acceleration
  // acceleration[0] = msg->linear_acceleration.x;
  // acceleration[1] = msg->linear_acceleration.y;
  // //get angular_velocity
  // angular_velocity = msg->angular_velocity.z;
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

  // icp
  point_solver.estimate_move(estimate_icp.angle,movement);
  estimate_icp.x += movement.x;
  estimate_icp.y += movement.y;
  // estimate_icp.angle += movement.angle;

  // ekf observe
  ekf_slam_solver.observe(point_solver);
}

void VelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v = msg->linear.x;
  double w = msg->angular.z;
  // ROS_INFO("Node listener heard vel: vel=%f,w=%f",v,w);

  // ekf predict
  ekf_slam_solver.predict(v,w,0.1);
  estimate_ekf.x = ekf_slam_solver.pose[0];
  estimate_ekf.y = ekf_slam_solver.pose[1];
  estimate_ekf.angle = ekf_slam_solver.pose[2];

  // odom predict
  odom_predict_solver.predict(v,w,0.1);
  estimate_predict.x = odom_predict_solver.pose[0];
  estimate_predict.y = odom_predict_solver.pose[1];
  estimate_predict.angle = odom_predict_solver.pose[2];
}

void publish_tf()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(estimate_ekf.x, estimate_ekf.y, 0.0));
  q.setRPY(0, 0, estimate_ekf.angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "ekf_estimator"));

  transform.setOrigin(tf::Vector3(estimate_predict.x, estimate_predict.y, 0.0));
  q.setRPY(0, 0, estimate_predict.angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "predict_estimator"));

  transform.setOrigin(tf::Vector3(estimate_icp.x, estimate_icp.y, 0.0));
  q.setRPY(0, 0, estimate_icp.angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "icp_estimator"));
  
  // ROS_INFO("---------------publish tf----------------");
  // ROS_INFO("ekf estimation: x=%f, y=%f, eular_angle=%f",estimate_ekf.x,estimate_ekf.y,estimate_ekf.angle);
  // ROS_INFO("predict estimation: x=%f, y=%f, eular_angle=%f",estimate_predict.x,estimate_predict.y,estimate_predict.angle);
  // ROS_INFO("icp estimation: x=%f, y=%f, eular_angle=%f",estimate_icp.x,estimate_icp.y,estimate_icp.angle);
  // ROS_INFO("real Odom: x=%f, y=%f, eular_angle=%f", current_pose.x, current_pose.y, current_pose.angle);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, OdomCallback);
  ros::Subscriber sub2 = n.subscribe("/imu", 1000, ImuCallback);
  ros::Subscriber sub3 = n.subscribe("/scan", 1000, ScanCallback);
  ros::Subscriber sub4 = n.subscribe("/cmd_vel", 1000, VelCallback);

  ros::Rate loop_rate(10);
  std::ofstream path_file, error_file;
  path_file.open("icp_path.txt");
  error_file.open("icp_error.txt");
  while (ros::ok()){
    publish_tf();
    error = pow(pow(estimate_icp.x-current_pose.x,2)+pow(estimate_icp.y-current_pose.y,2),0.5);
    path_file<<estimate_icp.x<<" "<<estimate_icp.y<<std::endl;
    error_file<<error<<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  path_file.close();
  error_file.close();
  return 0;
}