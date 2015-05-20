cmake_minimum_required(VERSION 2.8.3)
project(amazon_challenge_grasping)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()


add_service_files(
	FILES
	blindSeg.srv
 )


generate_messages(
  DEPENDENCIES std_msgs  # Or other packages containing msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
