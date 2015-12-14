cmake_minimum_required(VERSION 2.8.3)
project(correct_icp_mapper_pose)

find_package(catkin REQUIRED COMPONENTS
  ethzasl_icp_mapper
  geometry_msgs
  roscpp
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(correct_icp_mapper_pose src/CorrectPose.cpp)
add_executable(correct_world_odom src/CorrectWorldOdom.cpp)

target_link_libraries(correct_icp_mapper_pose ${catkin_LIBRARIES})
target_link_libraries(correct_world_odom ${catkin_LIBRARIES})

