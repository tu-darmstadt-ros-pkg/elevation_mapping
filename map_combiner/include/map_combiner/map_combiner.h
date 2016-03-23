#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


// Kindr
//#include <kindr/poses/PoseEigen.hpp>
//#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

// ROS (time)
#include <ros/ros.h>

namespace map_combiner {

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class MapCombiner
{
 public:

  /*!
   * Constructor.
   */
  MapCombiner()
  {}

};

} /* namespace */
