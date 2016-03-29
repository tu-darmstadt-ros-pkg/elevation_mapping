#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>
#include <map_combiner/MapCombinerConfig.h>


// Kindr
//#include <kindr/poses/PoseEigen.hpp>
//#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

// ROS (time)
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <cv_debug_provider/cv_debug_provider.h>



namespace map_combiner {

/*!
 * Combines grid map data from a premade static map and a rolling elevation map
 */
class MapCombiner
{
 public:

  /*!
   * Constructor.
   */
  MapCombiner();


  bool combineMaps();

protected:
  void staticMapCallback(const nav_msgs::OccupancyGrid& msg);
  void localElevationMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose);

  void dynRecParamCallback(map_combiner::MapCombinerConfig &config, uint32_t level);

  void publishFusedNavGrid();
  void callElevationMapReset();

  bool updateInflatedLayer(grid_map::GridMap& map);


  ros::Subscriber static_map_sub_;
  ros::Subscriber local_elevation_map_sub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber initial_pose_sub_;

  ros::Publisher fused_map_pub_;
  ros::Publisher fused_ros_map_pub_;

  ros::ServiceClient reset_elev_map_service_client_;


  grid_map::GridMap static_map_retrieved_;
  grid_map::GridMap static_map_fused_;

  //! Fused elevation map as grid map.
  grid_map::GridMap local_elevation_map_;

  geometry_msgs::PoseStampedConstPtr robot_pose_;

  double p_obstacle_diff_threshold_;
  double p_pose_height_offset_;
  bool p_fuse_elevation_map_;

  dynamic_reconfigure::Server<map_combiner::MapCombinerConfig> dyn_rec_server_;

  boost::shared_ptr<CvDebugProvider> debug_img_provider_;
};

} /* namespace */
