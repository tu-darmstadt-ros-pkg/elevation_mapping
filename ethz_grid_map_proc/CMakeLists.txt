cmake_minimum_required(VERSION 2.8.3)
project(ethz_grid_map_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  nav_msgs
  roscpp
  dynamic_reconfigure
  hector_obstacle_msgs
)

generate_dynamic_reconfigure_options(
  cfg/GridMapProc.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ethz_grid_map_proc
#  CATKIN_DEPENDS grid_map_ros nav_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/grid_map_to_occupancy_grid_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

