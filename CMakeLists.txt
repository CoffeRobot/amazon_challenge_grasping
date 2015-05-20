cmake_minimum_required(VERSION 2.8.3)
project(amazon_challenge_grasping)


find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs
)

add_service_files(
  FILES
  BaseMove.srv
)
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
 # CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
