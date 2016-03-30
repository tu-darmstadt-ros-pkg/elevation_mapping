#include <map_combiner/map_combiner.h>

#include <std_srvs/Empty.h>

#include <cv_image_proc/cv_image_convert.h>

#include <eigen_conversions/eigen_msg.h>

#include <grid_map_proc/grid_map_polygon_tools.h>

namespace map_combiner{

MapCombiner::MapCombiner()
{
  ros::NodeHandle pnh("~");

  dyn_rec_server_.setCallback(boost::bind(&MapCombiner::dynRecParamCallback, this, _1, _2));

  debug_img_provider_.reset(new CvDebugProvider(pnh, sensor_msgs::image_encodings::RGB8, true));

  poly_debug_pub_ = pnh.advertise<geometry_msgs::PolygonStamped>("debug_poly", 1, false);

  fused_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("/fused_grid_map", 1, false);
  fused_ros_map_pub_ = pnh.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 1, true);


  reset_elev_map_service_client_ = pnh.serviceClient<std_srvs::Empty>("/elevation_mapping/clear_map");

  initial_pose_sub_ = pnh.subscribe("/initialpose", 1, &MapCombiner::initialPoseCallback, this);

  pose_sub_ = pnh.subscribe("/robot_pose", 1, &MapCombiner::poseCallback, this);

  static_map_sub_ = pnh.subscribe("/map", 1, &MapCombiner::staticMapCallback, this);
  local_elevation_map_sub_ = pnh.subscribe("/elevation_mapping/elevation_map", 1, &MapCombiner::localElevationMapCallback, this);
}

void MapCombiner::staticMapCallback(const nav_msgs::OccupancyGrid& msg)
{
  grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", static_map_retrieved_);

  static_map_fused_ = static_map_retrieved_;


  this->updateInflatedLayer(static_map_retrieved_);
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
  ros::WallTime start_time = ros::WallTime::now();

  ROS_DEBUG("Combine started");
  if (!robot_pose_.get()){
    ROS_WARN("Cannot retrieve robot pose in combineMaps, aborting.");
    return false;
  }

  //const std::vector<std::string>& layers = local_elevation_map_.getLayers();
  //for (size_t i = 0; i < layers.size(); ++i)
  //  std::cout << layers[i] << "\n";


  //std::cout << "pose: " << robot_pose_->pose.position.x << " , " << robot_pose_->pose.position.y << "\n";

  //float robot_elevation = local_elevation_map_.atPosition("elevation", grid_map::Position(robot_pose_->pose.position.x, robot_pose_->pose.position.y));

  //if (std::isnan(robot_elevation)){
  float robot_elevation = robot_pose_->pose.position.z + p_pose_height_offset_;
  //}

  const grid_map::Length& local_length = local_elevation_map_.getLength();
  const grid_map::Position& local_position = local_elevation_map_.getPosition();

  //std::cout << "local_pos: " << local_position << " length: " << local_length << "\n";

  const grid_map::Length& static_length = static_map_retrieved_.getLength();
  const grid_map::Position& static_position = static_map_retrieved_.getPosition();

  //std::cout << "static_pos: " << static_position << " length: " << static_length << "\n";




  this->static_map_retrieved_.extendToInclude(local_elevation_map_);

  bool submap_create_success;
  grid_map::GridMap static_cut = this->static_map_retrieved_.getSubmap(local_position, local_length, submap_create_success);

  /*
  cv::Mat static_cut_mat;
  grid_map::GridMapRosConverter::toCvImage(static_cut, "occupancy", static_cut_mat);
  //debug_img_provider_->addDebugImage(static_cut_mat);

  cv::Mat static_cut_mat_gray;
  cv::cvtColor(static_cut_mat, static_cut_mat_gray, CV_BGRA2GRAY);



  int erosion_type = cv::MORPH_ELLIPSE;
  int erosion_size = 2;
  cv::Mat element = cv::getStructuringElement( erosion_type,
                                               cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               cv::Point( erosion_size, erosion_size ) );

  cv::Mat static_cut_mat_eroded;
  cv::dilate(static_cut_mat_gray, static_cut_mat_eroded, element);
  //debug_img_provider_->addDebugImage(static_cut_mat_eroded);

  //eroded = 255 - eroded;

  */



  const grid_map::Length& static_cut_length = static_cut.getLength();
  const grid_map::Position& static_cut_position = static_cut.getPosition();

  //std::cout << "static_cut_pos: " << static_cut_position << " length: " << static_cut_length << "\n";

  if (!submap_create_success){
    ROS_WARN("Submap creation failed, aborting.");
    return false;
  }

  grid_map::Matrix& elev_data   = local_elevation_map_["elevation"];
  grid_map::Matrix& static_cut_data = static_cut["occupancy"];
  grid_map::Matrix& static_cut_inflated_data = static_cut["occupancy_inflated"];

  //@TODO This can be made faster as described in https://github.com/ethz-asl/grid_map/issues/53
  for (grid_map::GridMapIterator iterator(static_cut); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);

    if (static_cut_inflated_data(index(0), index(1)) < 0.5){

      grid_map::Position position;
      static_cut.getPosition(index, position);

      grid_map::Index elev_index;
      local_elevation_map_.getIndex(position, elev_index);

      //std::cout << "re: " << robot_elevation << " elev: " << elev_data(index(0), index(1)) << "\n";

      //std::cout << "re: " << robot_elevation << " elev: " << static_cut_inflated_data(index(0), index(1)) << "\n";

      //if (static_data(index(0), index(1)) < 0.001){


      if ( std::abs( robot_elevation - elev_data(elev_index(0), elev_index(1)) ) > p_obstacle_diff_threshold_ ){
        static_cut_data(index(0), index(1)) = 100.0;
      }
    }
    //Uncomment below to mark everything occupied
    //static_cut_data(index(0), index(1)) = 100.0;

  }

  grid_map::Polygon pose_footprint = grid_map_polygon_tools::getTransformedPoly(footprint_poly_, robot_pose_->pose);

  //const std::vector<grid_map::Position>& vertices = pose_footprint.getVertices();

  //for (size_t i  = 0; i < vertices.size(); ++i){
  //  std::cout << "v: " << vertices[i].x() << " " <<  vertices[i].y() << "\n";
  //}

  if (poly_debug_pub_.getNumSubscribers() > 0){
    geometry_msgs::PolygonStamped poly_msg;
    grid_map::PolygonRosConverter::toMessage(pose_footprint, poly_msg);
    poly_debug_pub_.publish(poly_msg);
  }

  for (grid_map::PolygonIterator poly_iterator(static_cut, pose_footprint); !poly_iterator.isPastEnd(); ++poly_iterator) {
    const grid_map::Index index(*poly_iterator);

    //std::cout << "idx: " << index(0) << " " << index(1) << "\n";

    static_cut_data(index(0), index(1)) = 0.0;
  }


  static_map_fused_.addDataFrom(static_cut, true, true, true);

  if (fused_map_pub_.getNumSubscribers() > 0){
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(static_map_fused_, msg);
    fused_map_pub_.publish(msg);
  }

  this->publishFusedNavGrid();

  /*
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();
  grid_map::GridMapRosConverter::toOccupancyGrid(static_map_retrieved_, "occupancy_inflated", 0.0, 1.0, msg);
  fused_ros_map_pub_.publish(msg);
  */

  //debug_img_provider_->publishDebugImage();

  ROS_DEBUG("combineMaps took %f seconds", (ros::WallTime::now() - start_time).toSec());

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
  p_obstacle_diff_threshold_ = config.elev_diff_threshold;
  p_pose_height_offset_      = config.pose_height_offset;
  p_fuse_elevation_map_      = config.fuse_elevation_map;

  grid_map_polygon_tools::setFootprintPoly(config.footprint_x, config.footprint_y, this->footprint_poly_);

  ROS_INFO("MapCombiner params set: obstacle thresh: %f height offset: %f", p_obstacle_diff_threshold_,  p_pose_height_offset_);
}

bool MapCombiner::updateInflatedLayer(grid_map::GridMap& map)
{
  cv::Mat static_map_cv;
  grid_map::GridMapRosConverter::toCvImage(map, "occupancy", static_map_cv);

  //ROS_WARN("type: %d", static_map_cv.type());
  //debug_img_provider_->addDebugImage(static_cut_mat);

  cv::Mat static_map_cv_gray;
  cv::cvtColor(static_map_cv, static_map_cv_gray, CV_BGRA2GRAY);

  int erosion_type = cv::MORPH_ELLIPSE;
  int erosion_size = 2;
  cv::Mat element = cv::getStructuringElement( erosion_type,
                                               cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               cv::Point( erosion_size, erosion_size ) );

  cv_bridge::CvImage static_map_cv_eroded;
  static_map_cv_eroded.encoding = sensor_msgs::image_encodings::MONO8;


  cv::dilate(static_map_cv_gray, static_map_cv_eroded.image, element);

  //cv::Mat tmp;

  //cv::transpose(static_map_cv_eroded.image, tmp);
  // cv::flip(tmp, tmp, 0);
  //cv::flip(tmp, static_map_cv_eroded.image, 1);    // Flip on both axis at the same time.


  debug_img_provider_->addDebugImage(static_map_cv_eroded.image);
  debug_img_provider_->publishDebugImage();

  //cv::Mat static_map_cv_eroded_uc8;
  //cv_bridge::CvImage static_map_cv_eroded_uc8_image;

  //cv::cvtColor(static_map_cv_eroded, static_map_cv_eroded_uc8, CV_GRAY2BGRA);

  ROS_DEBUG("map size: %d %d grid size: %d %d, inflated size: %d %d",
           map.getSize()[0],
           map.getSize()[1],
           static_map_cv.size().width,
           static_map_cv.size().height,
           static_map_cv_eroded.image.size().width,
           static_map_cv_eroded.image.size().height);

  sensor_msgs::ImagePtr img = static_map_cv_eroded.toImageMsg();
  grid_map::GridMapRosConverter::addLayerFromImage(*img.get(), "occupancy_inflated", map);



  return true;
}

}
