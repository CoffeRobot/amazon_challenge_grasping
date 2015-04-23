cmake_minimum_required(VERSION 2.8.3)
project(amazon_challenge_grasping)


find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
