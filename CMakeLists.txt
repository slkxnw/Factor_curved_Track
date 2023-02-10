cmake_minimum_required(VERSION 3.0.2)
project(factor_curved_track)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  roscpp
  rospy
  std_msgs
  track_msgs
)

#Eigen3
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

#Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES factor_curved_track
 CATKIN_DEPENDS geometry_msgs message_filters roscpp rospy std_msgs track_msgs
 DEPENDS Boost EIGEN3 G2O CSparse
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
