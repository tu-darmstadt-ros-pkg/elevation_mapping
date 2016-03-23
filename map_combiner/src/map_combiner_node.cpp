
#include <ros/ros.h>
//#include "elevation_mapping/ElevationMapping.hpp"

#include <map_combiner/map_combiner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_combiner");
  ros::NodeHandle nodeHandle("~");
  map_combiner::MapCombiner mc;

  ros::spin();
  return 0;
}
