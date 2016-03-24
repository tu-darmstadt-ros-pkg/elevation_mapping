#include <map_combiner/map_combiner.h>


namespace map_combiner{

MapCombiner::MapCombiner()
{
  ros::NodeHandle pnh("~");
  static_map_sub_ = pnh.subscribe("/map", 1, &MapCombiner::staticMapCallback, this);
  local_elevation_map_sub_ = pnh.subscribe("/elevation_mapping/elevation_map", 1, &MapCombiner::localElevationMapCallback, this);

  pose_sub_ = pnh.subscribe("/robot_pose", 1, &MapCombiner::poseCallback, this);


  fused_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("/map_fused_grid_map", 1, true);
  fused_ros_map_pub_ = pnh.advertise<nav_msgs::OccupancyGrid>("map_fused", 1, true);
}

void MapCombiner::staticMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy", static_map_retrieved_);

  static_map_fused_ = static_map_retrieved_;
}

void MapCombiner::localElevationMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, local_elevation_map_);
  this->combineMaps();
}

void MapCombiner::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  robot_pose_ = msg;
}

bool MapCombiner::combineMaps()
{
  ROS_INFO("Combine started");
  if (!robot_pose_.get()){
    ROS_WARN("Cannot retrieve robot pose in combineMaps, aborting.");
    return false;
  }

  const std::vector<std::string>& layers = local_elevation_map_.getLayers();
  for (size_t i = 0; i < layers.size(); ++i)
    std::cout << layers[i] << "\n";


  std::cout << "pose: " << robot_pose_->pose.position.x << " , " << robot_pose_->pose.position.y << "\n";

  float robot_elevation = local_elevation_map_.atPosition("elevation", grid_map::Position(robot_pose_->pose.position.x, robot_pose_->pose.position.y));

  if (std::isnan(robot_elevation)){
    robot_elevation = robot_pose_->pose.position.z - 0.18;
  }

  const grid_map::Length& local_length = local_elevation_map_.getLength();
  const grid_map::Position& local_position = local_elevation_map_.getPosition();

  std::cout << "local_pos: " << local_position << " length: " << local_length << "\n";

  const grid_map::Length& static_length = static_map_retrieved_.getLength();
  const grid_map::Position& static_position = static_map_retrieved_.getPosition();

  std::cout << "static_pos: " << static_position << " length: " << static_length << "\n";




  this->static_map_retrieved_.extendToInclude(local_elevation_map_);

  bool submap_create_success;
  grid_map::GridMap static_cut = this->static_map_retrieved_.getSubmap(local_position, local_length, submap_create_success);



  const grid_map::Length& static_cut_length = static_cut.getLength();
  const grid_map::Position& static_cut_position = static_cut.getPosition();

  std::cout << "static_cut_pos: " << static_cut_position << " length: " << static_cut_length << "\n";

  if (!submap_create_success){
    ROS_WARN("Submap creation failed, aborting.");
    return false;
  }

  grid_map::Matrix& elev_data   = local_elevation_map_["elevation"];
  grid_map::Matrix& static_cut_data = static_cut["occupancy"];


  for (grid_map::GridMapIterator iterator(static_cut); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);

      grid_map::Position position;
      static_cut.getPosition(index, position);

      grid_map::Index elev_index;
      local_elevation_map_.getIndex(position, elev_index);

      //std::cout << "re: " << robot_elevation << " elev: " << elev_data(index(0), index(1)) << "\n";

      //if (static_data(index(0), index(1)) < 0.001){
        if ( std::abs( robot_elevation - elev_data(elev_index(0), elev_index(1)) ) > 0.15 ){
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

  if (fused_ros_map_pub_.getNumSubscribers() > 0){
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    grid_map::GridMapRosConverter::toOccupancyGrid(static_map_fused_, "occupancy", 0.0, 100.0, msg);
    fused_ros_map_pub_.publish(msg);
  }


  return true;
}

}
