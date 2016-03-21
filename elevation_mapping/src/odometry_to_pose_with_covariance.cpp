/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


class OdometryToPoseCovStamped
{
public:

  OdometryToPoseCovStamped()
  {
    ros::NodeHandle nodeHandle("~");

    pose_cov_pub_ = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_cov", 3, false);

    odom_sub_ = nodeHandle.subscribe("/ground_truth/state", 1, &OdometryToPoseCovStamped::odomCallback, this);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    geometry_msgs::PoseWithCovarianceStamped out_msg;
    out_msg.pose = msg->pose;
    out_msg.header = msg->header;

    pose_cov_pub_.publish(out_msg);
  }

protected:
  ros::Publisher pose_cov_pub_;
  ros::Subscriber odom_sub_;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_mapping");


  OdometryToPoseCovStamped odom_to_pose;

  ros::spin();
  return 0;
}
