#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <ethz_grid_map_proc/GridMapProcConfig.h>

class GridMapToOccupancyGrid
{
public:

  GridMapToOccupancyGrid(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh) ,pnh_(pnh)
  {
    dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    dyn_rec_server_->setCallback(boost::bind(&GridMapToOccupancyGrid::reconfigureCallback, this, _1, _2));  
      
      
    global_map_.add("obstacle");
    global_map_.add("traversability");
    global_map_.add("fused");
         //grid_map_.add("update_time");
    //global_map_.setGeometry(grid_map::Length(20.0, 20.0), 0.05);

    global_map_.setFrameId("world");
    
    this->clear();
    
    occ_grid_raw_pub_    = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map_raw", 1);
    occ_grid_pub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map", 1);
    global_occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/global_map_debug", 1);


    obstacle_grid_map_sub_ = nh_.subscribe("/obstacle_map_throttled", 1, &GridMapToOccupancyGrid::obstacleMapCallback, this);
    
    grid_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &GridMapToOccupancyGrid::gridMapCallback, this);
    syscommand_sub_ = nh_.subscribe("/syscommand", 1, &GridMapToOccupancyGrid::sysCommandCallback, this);
    path_sub_ = nh_.subscribe("/path_to_follow", 1, &GridMapToOccupancyGrid::pathCallback, this);
  }
      
  void obstacleMapCallback(const nav_msgs::OccupancyGridConstPtr msg)
  {
    grid_map::GridMap local_obstacle_map;
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "obstacle", local_obstacle_map);
    std::vector<std::string> layers_to_use;
    layers_to_use.push_back("obstacle");
    global_map_.addDataFrom(local_obstacle_map, true, true, false, layers_to_use);

    // fuse obstacle map with traversability map
    if (p_fusing_enabled_) {
      global_map_["fused"] = global_map_["obstacle"].cwiseMax(global_map_["traversability"]);
    } else {
      global_map_["fused"] = global_map_["obstacle"];
    }
    // publish fused map
    if (global_occ_grid_pub_.getNumSubscribers() > 0) {
      nav_msgs::OccupancyGrid occupancy_grid;
      grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, "fused", 0.0, 100.0, occupancy_grid);
      global_occ_grid_pub_.publish(occupancy_grid);
    }

    if (grid_map_pub_.getNumSubscribers() >0 )
    {
      grid_map_msgs::GridMap grid_map_msg;
      grid_map::GridMapRosConverter::toMessage(global_map_, grid_map_msg);
      grid_map_pub_.publish(grid_map_msg);
    }
  }
    
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr msg)
  {
    grid_map::GridMap local_grid_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, local_grid_map);
//    ROS_INFO_STREAM("map: " << local_grid_map["traversability"]);
    
    // Added because the local map is thresholded below
    if (occ_grid_raw_pub_.getNumSubscribers() > 0){
      nav_msgs::OccupancyGrid local_occupancy_grid;
      
      grid_map::GridMapRosConverter::toOccupancyGrid(local_grid_map, "traversability", 1.0, 0.0, local_occupancy_grid);
      occ_grid_raw_pub_.publish(local_occupancy_grid);
    }
    

    // Threshold map and convert traversability to obstacle
    grid_map::Matrix& data = local_grid_map["traversability"];
    grid_map::Matrix free = grid_map::Matrix::Zero(data.rows(), data.cols());
    grid_map::Matrix occupied = grid_map::Matrix::Constant(data.rows(), data.cols(), 100);
    data = (data.array() < p_occupied_threshold_).select(occupied, free); // also inverts matrix
    
    // Insert current local traversability map into global map
    std::vector<std::string> layers_to_use;
    layers_to_use.push_back("traversability");
    global_map_.addDataFrom(local_grid_map, true, true, false, layers_to_use);

    // Clear waypoints
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      grid_map::Position pos(path.poses[i].pose.position.x, path.poses[i].pose.position.y);

      for (grid_map::CircleIterator iterator(global_map_, pos, p_goal_clear_radius_);
             !iterator.isPastEnd(); ++iterator) {
      global_map_.at("traversability", *iterator) = 0.0;
      }

    }

    // Create obstacle U so robot drives forward/backward
    if (p_obstacle_u_forward_ || p_obstacle_u_backward_) {

          grid_map::Position left_top    (p_obstacle_u_size_, p_obstacle_u_size_);
          grid_map::Position left_bottom(-p_obstacle_u_size_, p_obstacle_u_size_);

          for (grid_map::LineIterator iterator(global_map_,  left_top, left_bottom);
              !iterator.isPastEnd(); ++iterator) {
            global_map_.at("traversability", *iterator) = 100.0;
          }

          grid_map::Position right_top    (p_obstacle_u_size_, -p_obstacle_u_size_);
          grid_map::Position right_bottom(-p_obstacle_u_size_, -p_obstacle_u_size_);

          for (grid_map::LineIterator iterator(global_map_, right_top, right_bottom);
              !iterator.isPastEnd(); ++iterator) {
            global_map_.at("traversability", *iterator) = 100.0;
          }

          if (p_obstacle_u_forward_) {
              for (grid_map::LineIterator iterator(global_map_, left_top, right_top);
                  !iterator.isPastEnd(); ++iterator) {
                global_map_.at("traversability", *iterator) = 100.0;
              }
          }

          if (p_obstacle_u_backward_) {
              for (grid_map::LineIterator iterator(global_map_, left_bottom, right_bottom);
                  !iterator.isPastEnd(); ++iterator) {
                global_map_.at("traversability", *iterator) = 100.0;
              }
          }

    }

    if (grid_map_pub_.getNumSubscribers() >0 )
    {
      grid_map_msgs::GridMap grid_map_msg;
      grid_map::GridMapRosConverter::toMessage(global_map_, grid_map_msg);
      grid_map_pub_.publish(grid_map_msg);
    }


    if (occ_grid_pub_.getNumSubscribers() > 0){
      nav_msgs::OccupancyGrid local_occupancy_grid;
      
      grid_map::GridMapRosConverter::toOccupancyGrid(local_grid_map, "traversability", 0.0, 100.0, local_occupancy_grid);
      occ_grid_pub_.publish(local_occupancy_grid);
    }
    
//    if (global_occ_grid_pub_.getNumSubscribers() > 0){
        
//      nav_msgs::OccupancyGrid occupancy_grid;
      
//      grid_map::GridMapRosConverter::toOccupancyGrid(global_map_, "traversability", 1.0, 0.0, occupancy_grid);
//      global_occ_grid_pub_.publish(occupancy_grid);
//    }
  }
  
  void pathCallback(const nav_msgs::PathConstPtr msg)
  {
    path = *msg;
  }

  void sysCommandCallback(const std_msgs::StringConstPtr msg)
  {
    if (msg->data == "reset"){
      this->clear();
    }
  }
  
  void clear()
  {
    global_map_.clearAll();
  }
  
  void reconfigureCallback(ethz_grid_map_proc::GridMapProcConfig &config, uint32_t level)
  {
    p_occupied_threshold_ = config.occupied_threshold;
    p_goal_clear_radius_ = config.goal_clear_radius;
    p_obstacle_u_forward_ = config.obstacle_u_forward;
    p_obstacle_u_backward_ = config.obstacle_u_backward;
    p_obstacle_u_size_ = config.obstacle_u_size;
    p_fusing_enabled_ = config.fusing_enabled;
    
    ROS_INFO("Reconfigure Request. Occupied threshold: %f clear radius:%f, of: %s ,ob: %s, fusing: %s", p_occupied_threshold_, p_goal_clear_radius_,
             config.obstacle_u_forward?"True":"False",
             config.obstacle_u_backward?"True":"False",
             config.fusing_enabled?"True":"False");
  }
  

private:
  ros::Publisher occ_grid_pub_;
  ros::Publisher occ_grid_raw_pub_;
  ros::Publisher global_occ_grid_pub_;

  ros::Publisher grid_map_pub_;
    
  ros::Subscriber obstacle_grid_map_sub_;
  ros::Subscriber grid_map_sub_;
  ros::Subscriber syscommand_sub_;
  ros::Subscriber path_sub_;
  

  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;


  grid_map::GridMap global_map_;
  
  typedef dynamic_reconfigure::Server<ethz_grid_map_proc::GridMapProcConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> dyn_rec_server_;
  boost::recursive_mutex config_mutex_;
  
  float p_occupied_threshold_;
  float p_goal_clear_radius_;
  bool p_obstacle_u_forward_;
  bool p_obstacle_u_backward_;
  float p_obstacle_u_size_;
  bool p_fusing_enabled_;
  
  nav_msgs::Path path;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grid_map_to_occupancy_grid_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  GridMapToOccupancyGrid grid_map_to_occ_grid(nh, pnh);

  ros::spin();

  return 0;
}






