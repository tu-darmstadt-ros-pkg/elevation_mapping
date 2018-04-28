#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

class GridMapToOccupancyGrid
{
public:

  void gridMapCallback(const grid_map_msgs::GridMapConstPtr msg)
  {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    nav_msgs::OccupancyGrid occupancy_grid;
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "traversability", 1.0, 0.0, occupancy_grid);
    occ_grid_pub_.publish(occupancy_grid);
  }

  GridMapToOccupancyGrid(ros::NodeHandle nh) : nh_(nh)
  {
    grid_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &GridMapToOccupancyGrid::gridMapCallback, this);
    occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map2", 1);
  }

private:
  ros::Subscriber grid_map_sub_;
  ros::Publisher occ_grid_pub_;
  ros::NodeHandle nh_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grid_map_to_occupancy_grid_node");
  ros::NodeHandle nh;

  GridMapToOccupancyGrid grid_map_to_occ_grid(nh);

  ros::spin();

  return 0;

}






