cmake_minimum_required(VERSION 3.0.2)
project(ekf_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  Eigen3 REQUIRED
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekf_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(listener src/listener.cpp src/point_solver.cpp src/ekf_slam_solver.cpp src/icp.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})