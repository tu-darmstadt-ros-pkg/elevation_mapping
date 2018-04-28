#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>

class GridMapToOccupancyGrid
{
public:

  GridMapToOccupancyGrid(ros::NodeHandle nh) : nh_(nh)
  {
    global_traversability_map_.add("traversability");
         //grid_map_.add("update_time");
    global_traversability_map_.setGeometry(grid_map::Length(20.0, 20.0), 0.05);
    global_traversability_map_.setFrameId("world");
    
    //@TODO: Clear global traversability map
    this->clear();

    
    occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map", 1);
    global_occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_traversability_map", 1);


    grid_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &GridMapToOccupancyGrid::gridMapCallback, this);
    syscommand_sub_ = nh_.subscribe("/syscommand", 1, &GridMapToOccupancyGrid::sysCommandCallback, this);
    
    
  }
      
    
  void gridMapCallback(const grid_map_msgs::GridMapConstPtr msg)
  {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());



    nav_msgs::OccupancyGrid occupancy_grid;
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    // Insert current local traversability map into global map
    std::vector<std::string> layers_to_use;
    layers_to_use.push_back("traversability");
    global_traversability_map_.addDataFrom(map, true, true, false, layers_to_use);

    grid_map::GridMapRosConverter::toOccupancyGrid(map, "traversability", 1.0, 0.0, occupancy_grid);
    occ_grid_pub_.publish(occupancy_grid);
  }

  void sysCommandCallback(const std_msgs::StringConstPtr msg)
  {
    if (msg->data == "reset"){
      this->clear();
    }
  };
  
  void clear()
  {
      //Clear global traversability map to be completely free space   
  }
  

private:
  ros::Publisher occ_grid_pub_;
  ros::Publisher global_occ_grid_pub_;  
    
  ros::Subscriber grid_map_sub_;
  ros::Subscriber syscommand_sub_;
  

  
  ros::NodeHandle nh_;


  grid_map::GridMap global_traversability_map_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grid_map_to_occupancy_grid_node");
  ros::NodeHandle nh;

  GridMapToOccupancyGrid grid_map_to_occ_grid(nh);

  ros::spin();

  return 0;

}






