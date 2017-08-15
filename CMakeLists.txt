cmake_minimum_required(VERSION 2.8.3)
project(wilson_ros CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_runtime
  nav_msgs
  nodelet
  openslam_gmapping
  rosbag_storage
  roscpp
  std_msgs
  tf
)

# Necessary in order for gmapping to work
find_package(Boost REQUIRED signals)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
	std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wilson_ros
#  CATKIN_DEPENDS geometry_msgs map_server ros_stage roscpp rospy std_msgs tf
#  DEPENDS system_lib
)
catkin_package(
#    INCLUDE_DIRS include
    DEPENDS roscpp std_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# Boost is required for gmapping
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include_directories(src/gmapping)

add_executable(slam_gmapping src/gmapping/slam_gmapping.cpp src/gmapping/main.cpp)
target_link_libraries(slam_gmapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS slam_gmapping #slam_gmapping_replay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)