cmake_minimum_required(VERSION 3.0.2)
project(factor_curved_track)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -DPCL_NO_PRECOMPILE")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_DEBUG_POSTFIX "_d")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  roscpp
  rospy
  std_msgs
  track_msgs
)


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake) 
# message(${PROJECT_SOURCE_DIR})

#Eigen3
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

#Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
# message(${Boost_INCLUDE_DIRS})
# message(${EIGEN3_INCLUDE_DIRS})

#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES factor_curved_track
 CATKIN_DEPENDS geometry_msgs message_filters roscpp rospy std_msgs track_msgs
 DEPENDS Boost EIGEN3 G2O CSparse
)
# TODO这里直接给出eigen3的头文件路径，find_package失败
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  "/usr/include/eigen3"
)

set(THIRD_PARTY_LIBS
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${CSAPRSE_LIBRARY}
  g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
  ${GLOG_LIBRARIES})

add_subdirectory(src)