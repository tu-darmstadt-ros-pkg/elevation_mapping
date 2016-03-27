#include <map_combiner/map_combiner.h>

#include <std_srvs/Empty.h>

namespace map_combiner{

MapCombiner::MapCombiner()
{
  ros::NodeHandle pnh("~");

  dyn_rec_server_.setCallback(boost::bind(&MapCombiner::dynRecParamCallback, this, _1, _2));

  pose_sub_ = pnh.subscribe("/robot_pose", 1, &MapCombiner::poseCallback, this);


  fused_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("/fused_grid_map", 1, false);
  fused_ros_map_pub_ = pnh.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 1, true);

  static_map_sub_ = pnh.subscribe("/map", 1, &MapCombiner::staticMapCallback, this);
  local_elevation_map_sub_ = pnh.subscribe("/elevation_mapping/elevation_map", 1, &MapCombiner::localElevationMapCallback, this);

  initial_pose_sub_ = pnh.subscribe("/initialpose", 1, &MapCombiner::initialPoseCallback, this);

  reset_elev_map_service_client_ = pnh.serviceClient<std_srvs::Empty>("/elevation_mapping/clear_map");
}

void MapCombiner::staticMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy", static_map_retrieved_);

  static_map_fused_ = static_map_retrieved_;

  // Elevation map is reset as is assumed on new static map also
  // elevation data is outdated (i.e. floor change)
  this->callElevationMapReset();
  this->publishFusedNavGrid();
}

void MapCombiner::localElevationMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  if (p_fuse_elevation_map_){
    grid_map::GridMapRosConverter::fromMessage(*msg, local_elevation_map_);

    this->combineMaps();
  }
}

void MapCombiner::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  robot_pose_ = msg;
}

bool MapCombiner::combineMaps()
{
  ROS_DEBUG("Combine started");
  if (!robot_pose_.get()){
    ROS_WARN("Cannot retrieve robot pose in combineMaps, aborting.");
    return false;
  }

  //const std::vector<std::string>& layers = local_elevation_map_.getLayers();
  //for (size_t i = 0; i < layers.size(); ++i)
  //  std::cout << layers[i] << "\n";


  //std::cout << "pose: " << robot_pose_->pose.position.x << " , " << robot_pose_->pose.position.y << "\n";

  float robot_elevation = local_elevation_map_.atPosition("elevation", grid_map::Position(robot_pose_->pose.position.x, robot_pose_->pose.position.y));

  if (std::isnan(robot_elevation)){
    robot_elevation = robot_pose_->pose.position.z - 0.18;
  }

  const grid_map::Length& local_length = local_elevation_map_.getLength();
  const grid_map::Position& local_position = local_elevation_map_.getPosition();

  //std::cout << "local_pos: " << local_position << " length: " << local_length << "\n";

  const grid_map::Length& static_length = static_map_retrieved_.getLength();
  const grid_map::Position& static_position = static_map_retrieved_.getPosition();

  //std::cout << "static_pos: " << static_position << " length: " << static_length << "\n";




  this->static_map_retrieved_.extendToInclude(local_elevation_map_);

  bool submap_create_success;
  grid_map::GridMap static_cut = this->static_map_retrieved_.getSubmap(local_position, local_length, submap_create_success);



  const grid_map::Length& static_cut_length = static_cut.getLength();
  const grid_map::Position& static_cut_position = static_cut.getPosition();

  //std::cout << "static_cut_pos: " << static_cut_position << " length: " << static_cut_length << "\n";

  if (!submap_create_success){
    ROS_WARN("Submap creation failed, aborting.");
    return false;
  }

  grid_map::Matrix& elev_data   = local_elevation_map_["elevation"];
  grid_map::Matrix& static_cut_data = static_cut["occupancy"];

  //@TODO This can be made faster as described in https://github.com/ethz-asl/grid_map/issues/53
  for (grid_map::GridMapIterator iterator(static_cut); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);

      grid_map::Position position;
      static_cut.getPosition(index, position);

      grid_map::Index elev_index;
      local_elevation_map_.getIndex(position, elev_index);

      //std::cout << "re: " << robot_elevation << " elev: " << elev_data(index(0), index(1)) << "\n";

      //if (static_data(index(0), index(1)) < 0.001){
        if ( std::abs( robot_elevation - elev_data(elev_index(0), elev_index(1)) ) > p_pos_obstacle_diff_threshold_ ){
          static_cut_data(index(0), index(1)) = 100.0;
        }

      //}
  }


  static_map_fused_.addDataFrom(static_cut, true, true, true);

  if (fused_map_pub_.getNumSubscribers() > 0){
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(static_map_fused_, msg);
    fused_map_pub_.publish(msg);
  }

  this->publishFusedNavGrid();

  return true;
}

void MapCombiner::publishFusedNavGrid()
{
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  grid_map::GridMapRosConverter::toOccupancyGrid(static_map_fused_, "occupancy", 0.0, 100.0, msg);
  fused_ros_map_pub_.publish(msg);
}

void MapCombiner::callElevationMapReset()
{
  std_srvs::Empty srv;
  if (reset_elev_map_service_client_.call(srv)){
    ROS_INFO("Succesfully called reset elevation map service from map_combiner");
  }else{
    ROS_WARN("Failed to call reset elevation map service from map_combiner!");
  }
}

void MapCombiner::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  ROS_INFO("Received intialpose, resetting map_combiner map");

  static_map_fused_ = static_map_retrieved_;

  this->callElevationMapReset();
  this->publishFusedNavGrid();
}

void MapCombiner::dynRecParamCallback(map_combiner::MapCombinerConfig &config, uint32_t level)
{
  p_pos_obstacle_diff_threshold_ = config.pos_elev_diff_threshold;
  p_neg_obstacle_diff_threshold_ = config.neg_elev_diff_threshold;
  p_fuse_elevation_map_          = config.fuse_elevation_map;

  ROS_INFO("MapCombiner params set: pos thresh: %f neg thresh: %f", p_pos_obstacle_diff_threshold_,  p_neg_obstacle_diff_threshold_);
}

}
