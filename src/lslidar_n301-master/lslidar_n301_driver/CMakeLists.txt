cmake_minimum_required(VERSION 2.8.3)
project(lslidar_n301_driver)

add_definitions(-std=c++0x)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  nodelet

  lslidar_n301_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_n301_driver
  CATKIN_DEPENDS
    roscpp diagnostic_updater nodelet
    lslidar_n301_msgs
  DEPENDS
    boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Leishen n301 lidar driver
add_library(lslidar_n301_driver
  src/lslidar_n301_driver.cc
)
target_link_libraries(lslidar_n301_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_n301_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen n301 lidar nodelet
#add_library(lslidar_n301_driver_nodelet
#  src/lslidar_n301_driver_nodelet.cc
#)
#target_link_libraries(lslidar_n301_driver_nodelet
#  lslidar_n301_driver
#  ${catkin_LIBRARIES}
#)
#add_dependencies(lslidar_n301_driver_nodelet
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

# Leishen n301 lidar node
add_executable(lslidar_n301_driver_node
  src/lslidar_n301_driver_node.cc
)
target_link_libraries(lslidar_n301_driver_node
  lslidar_n301_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_n301_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
