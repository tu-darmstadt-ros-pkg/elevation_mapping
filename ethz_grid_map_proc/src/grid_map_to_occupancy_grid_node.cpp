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
      
      
    global_traversability_map_.add("traversability");
         //grid_map_.add("update_time");
    global_traversability_map_.setGeometry(grid_map::Length(20.0, 20.0), 0.05);
    global_traversability_map_.setFrameId("world");
    
    //@TODO: Implement clearing of global traversability map
    this->clear();

    
    occ_grid_pub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map", 1);
    global_occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);


    grid_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &GridMapToOccupancyGrid::gridMapCallback, this);
    syscommand_sub_ = nh_.subscribe("/syscommand", 1, &GridMapToOccupancyGrid::sysCommandCallback, this);
    path_sub_ = nh_.subscribe("/path_to_follow", 1, &GridMapToOccupancyGrid::pathCallback, this);
    
    
  }
      
    
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr msg)
  {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());


    grid_map::GridMap local_grid_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, local_grid_map);

    // Threshold map
    grid_map::Matrix& data = local_grid_map["traversability"];
    grid_map::Matrix zeros = data; zeros.setZero();
    grid_map::Matrix ones = data; ones.setOnes();
    data = (data.array() < p_occupied_threshold_).select(zeros, ones);
    
    // Insert current local traversability map into global map
    std::vector<std::string> layers_to_use;
    layers_to_use.push_back("traversability");
    global_traversability_map_.addDataFrom(local_grid_map, true, true, false, layers_to_use);

    if (occ_grid_pub_.getNumSubscribers() > 0){
      nav_msgs::OccupancyGrid local_occupancy_grid;
      
      grid_map::GridMapRosConverter::toOccupancyGrid(local_grid_map, "traversability", 1.0, 0.0, local_occupancy_grid);
      occ_grid_pub_.publish(local_occupancy_grid);
    }
    
    if (global_occ_grid_pub_.getNumSubscribers() > 0){
        
        
        
        
      nav_msgs::OccupancyGrid occupancy_grid;
      
      grid_map::GridMapRosConverter::toOccupancyGrid(global_traversability_map_, "traversability", 1.0, 0.0, occupancy_grid);
      global_occ_grid_pub_.publish(occupancy_grid);  
    }
  }
  
  void pathCallback(const nav_msgs::PathConstPtr msg)
  {
    path = *msg;
  };

  void sysCommandCallback(const std_msgs::StringConstPtr msg)
  {
    if (msg->data == "reset"){
      this->clear();
    }
  };
  
  void clear()
  {
      //Clear global traversability map to be completely free space
    global_traversability_map_["traversability"].setOnes();   
  }
  
  void reconfigureCallback(ethz_grid_map_proc::GridMapProcConfig &config, uint32_t level) {
      
    
    
    p_occupied_threshold_ = config.occupied_threshold;
    
    ROS_INFO("Reconfigure Request. Occupied threshold: %f ", p_occupied_threshold_);
  //ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
  //          config.int_param, config.double_param, 
  //          config.str_param.c_str(), 
  //          config.bool_param?"True":"False", 
  //          config.size);
}
  

private:
  ros::Publisher occ_grid_pub_;
  ros::Publisher global_occ_grid_pub_;  
    
  ros::Subscriber grid_map_sub_;
  ros::Subscriber syscommand_sub_;
  ros::Subscriber path_sub_;
  

  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;


  grid_map::GridMap global_traversability_map_;
  
  typedef dynamic_reconfigure::Server<ethz_grid_map_proc::GridMapProcConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> dyn_rec_server_;
  boost::recursive_mutex config_mutex_;
  
  float p_occupied_threshold_;
  
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






