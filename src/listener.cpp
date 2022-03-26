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
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"

// const std::vector<double> CONST_OBSTACLE = {-2,-2,-2,-1,-2,0,-2,1,-2,2, -1,-2,-1,-1,-1,0,-1,1,-1,2, 0,-2,0,-1,0,1,0,2, 1,-2,1,-1,1,0,1,1,1,2, 2,-2,2,-1,2,0,2,1,2,2};
const std::vector<double> CONST_OBSTACLE = {-2,-2,-2,-1,-2,0,-2,1,-2,2, -1,-2,-1,-1,-1,0,-1,1,-1,2, 0,-2,0,-1,0,1,0,2, 1,-2,1,-1,1,0,1,1,1,2, 2,-2,2,-1,2,0,2,1,2,2};

CoordPoint current_pose(3,3,0);
CoordPoint estimate_icp(3,3,0);
CoordPoint estimate_ekf(3,3,0);
PointSolver point_solver;
EkfSlamSolver ekf_slam_solver(3,3,0,CONST_OBSTACLE);
std::vector<double> obstacle_pose;
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
  // point_solver.estimate_obstacle(current_pose.x,current_pose.y,current_pose.angle,0.05,obstacle_pose,true);
  // ekf observe
  ekf_slam_solver.observe(point_solver);
}

void VelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double v = msg->linear.x;
  double w = msg->angular.z;
  // ROS_INFO("Node listener heard vel: vel=%f,w=%f",v,w);

  // ekf predict
  ekf_slam_solver.predict(v,w,0.05);
  estimate_ekf.x = ekf_slam_solver.pose[0];
  estimate_ekf.y = ekf_slam_solver.pose[1];
  estimate_ekf.angle = ekf_slam_solver.pose[2];
  obstacle_pose = ekf_slam_solver.obstacle_pose;
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

  transform.setOrigin(tf::Vector3(estimate_icp.x, estimate_icp.y, 0.0));
  q.setRPY(0, 0, estimate_icp.angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "icp_estimator"));
  
  // ROS_INFO("---------------publish tf----------------");
  ROS_INFO("ekf estimation: x=%f, y=%f, eular_angle=%f",estimate_ekf.x,estimate_ekf.y,estimate_ekf.angle);
  ROS_INFO("icp estimation: x=%f, y=%f, eular_angle=%f",estimate_icp.x,estimate_icp.y,estimate_icp.angle);
  ROS_INFO("real Odom: x=%f, y=%f, eular_angle=%f", current_pose.x, current_pose.y, current_pose.angle);
}

void publish_obstacle(ros::Publisher & vis_obstacle_pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.g = 1.0;
  for(size_t i=0;i<obstacle_pose.size()/2;i++){
    marker.id = i;
    marker.pose.position.x = obstacle_pose[2*i];
    marker.pose.position.y = obstacle_pose[2*i+1];

    marker_array.markers.push_back(marker);
  }
  vis_obstacle_pub.publish(marker_array);
}

void publish_model_state(ros::Publisher & model_pose_pub)
{
    geometry_msgs::Pose vehicle_pose_pub;
    geometry_msgs::Pose obstacle_pose_pub;
    gazebo_msgs::ModelStates msg;
    
    //update vehicle pose
    vehicle_pose_pub.position.x = estimate_ekf.x;
    vehicle_pose_pub.position.y = estimate_ekf.y;
    vehicle_pose_pub.position.z = estimate_ekf.angle; //attention! some tricks here
    msg.pose.push_back(vehicle_pose_pub);

    //update obstacle pose
    for(size_t i=0;i<CONST_OBSTACLE.size()/2;i++){
      obstacle_pose_pub.position.x = CONST_OBSTACLE[i*2]+3;
      obstacle_pose_pub.position.y = CONST_OBSTACLE[i*2+1]+3;
      msg.pose.push_back(obstacle_pose_pub);
    }
    model_pose_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, OdomCallback);
  ros::Subscriber sub2 = n.subscribe("/imu", 1000, ImuCallback);
  ros::Subscriber sub3 = n.subscribe("/scan", 1000, ScanCallback);
  ros::Subscriber sub4 = n.subscribe("/cmd_vel", 1000, VelCallback);
  ros::Publisher model_pose_pub = n.advertise<gazebo_msgs::ModelStates>("Environment", 1000);
  ros::Publisher vis_obstacle_pub = n.advertise<visualization_msgs::MarkerArray>("vis_obstacle", 10);

  ros::Rate loop_rate(10);
  std::ofstream path_file, error_file;
  std::ofstream path_file2;
  path_file.open("real_path.txt");
  path_file2.open("ekf_path.txt");
  error_file.open("ekf_error.txt");
  while (ros::ok()){
    publish_model_state(model_pose_pub);    
    publish_tf();
    publish_obstacle(vis_obstacle_pub);
    error = pow(pow(estimate_ekf.x-current_pose.x,2)+pow(estimate_ekf.y-current_pose.y,2),0.5);
    path_file<<current_pose.x<<" "<<current_pose.y<<std::endl;
    path_file2<<estimate_ekf.x<<" "<<estimate_ekf.y<<std::endl;
    error_file<<error<<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  path_file.close();
  path_file2.close();
  error_file.close();
  return 0;
}