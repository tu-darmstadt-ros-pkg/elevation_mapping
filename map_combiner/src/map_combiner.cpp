#include <map_combiner/map_combiner.h>

#include <std_srvs/Empty.h>

#include <cv_image_proc/cv_image_convert.h>

#include <eigen_conversions/eigen_msg.h>

#include <grid_map_proc/grid_map_polygon_tools.h>

#include <visualization_msgs/MarkerArray.h>
#include <hector_worldmodel_msgs/PosePercept.h>

namespace map_combiner{

MapCombiner::MapCombiner():
    pnh("~")
{


    debug_img_provider_.reset(new CvDebugProvider(pnh, sensor_msgs::image_encodings::RGB8, true));

    ros::NodeHandle flood_debug_nh("~/flood_fill_");
    flood_debug_img_provider_.reset(new CvDebugProvider(flood_debug_nh, sensor_msgs::image_encodings::RGB8, true));

    pose_percept_publisher_ = pnh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 5, false);pose_percept_publisher_ = pnh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 5, false);

    poly_debug_pub_ = pnh.advertise<geometry_msgs::PolygonStamped>("debug_poly", 1, false);

    obstacle_poly_pub_  = pnh.advertise<geometry_msgs::PolygonStamped>("obstacle_poly", 1, false);
    obstacle_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 1, false);

    fused_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("/fused_grid_map", 1, false);
    fused_ros_map_pub_ = pnh.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 1, true);


    reset_elev_map_service_client_ = pnh.serviceClient<std_srvs::Empty>("/elevation_mapping/clear_map");

    initial_pose_sub_ = pnh.subscribe("/initialpose", 1, &MapCombiner::initialPoseCallback, this);

    pose_sub_ = pnh.subscribe("/robot_pose", 1, &MapCombiner::poseCallback, this);
    collision_pose_sub_ = pnh.subscribe("/collision_pose", 1, &MapCombiner::collisionPoseCallback, this);

    static_map_sub_ = pnh.subscribe("/map", 1, &MapCombiner::staticMapCallback, this);
    local_elevation_map_sub_ = pnh.subscribe("/elevation_mapping/elevation_map", 1, &MapCombiner::localElevationMapCallback, this);

   // worldmodel_sub_ = pnh.subscribe("/worldmodel/objects", 40, &MapCombiner::worldmodelCallback, this);


    dyn_rec_server_.setCallback(boost::bind(&MapCombiner::dynRecParamCallback, this, _1, _2));

    obstacle_marker_box_.type = visualization_msgs::Marker::CUBE;
    obstacle_marker_box_.action = visualization_msgs::Marker::ADD;
    obstacle_marker_box_.header.frame_id = "world";
    obstacle_marker_box_.id = 0;

    obstacle_marker_text_ = obstacle_marker_box_;
    obstacle_marker_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    obstacle_marker_text_.scale.z = 0.15;
    obstacle_marker_text_.id = 1;
}

void MapCombiner::staticMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", static_map_retrieved_);

    static_map_fused_ = static_map_retrieved_;


    this->updateInflatedLayer(static_map_retrieved_);
    // Elevation map is reset as is assumed on new static map also
    // elevation data is outdated (i.e. floor change)
    //this->callElevationMapReset();

    if (!p_fuse_elevation_map_)
    {
        this->publishFusedNavGrid();
    }
}

void MapCombiner::localElevationMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
    grid_map::GridMapRosConverter::fromMessage(*msg, local_elevation_map_);

    if (p_fuse_elevation_map_){
        this->combineMaps();
    }

}

void MapCombiner::worldmodelCallback(const hector_worldmodel_msgs::ObjectModel& msg)
{
    /*
    size_t size = msg.objects.size();

    // Use static map always as base as we otherwise might clear step obstacles and start
    // planning through them
    static_map_fused_ = static_map_retrieved_;

    //Add confirmed obstacles and heat sources as occupied
    for (size_t i = 0; i < size; ++i){
        const hector_worldmodel_msgs::Object& obj = msg.objects[i];

        double obstacle_radius = -1.0;

        if (obj.info.class_id == "obstacle"){
            if (obj.state.state == hector_worldmodel_msgs::ObjectState::CONFIRMED){
                obstacle_radius = p_large_obstacle_radius_;
            }
            //}else if (obj.state.state == hector_worldmodel_msgs::ObjectState::ACTIVE){
            //  obstacle_radius = p_small_obstacle_radius_;
            //}
        }else if (obj.info.class_id == "heat_source"){
            if (obj.state.state == hector_worldmodel_msgs::ObjectState::CONFIRMED){
                obstacle_radius = p_large_obstacle_radius_;
            }
        }

        if ( obstacle_radius > 0.0){

            ROS_INFO("worldmodelCallback");
            grid_map::Matrix& static_map_data = static_map_fused_["occupancy"];

            grid_map::Position position (obj.pose.pose.position.x, obj.pose.pose.position.y);

            for (grid_map::CircleIterator circle_iterator(static_map_fused_, position, obstacle_radius); !circle_iterator.isPastEnd(); ++circle_iterator) {
                const grid_map::Index index(*circle_iterator);

                static_map_data(index(0), index(1)) = 100.0;
            }
        }
    }


    //Clear footprint
    if (robot_pose_.get()){

        grid_map::Polygon pose_footprint = grid_map_polygon_tools::getTransformedPoly(footprint_poly_, robot_pose_->pose);

        if (poly_debug_pub_.getNumSubscribers() > 0){
            geometry_msgs::PolygonStamped poly_msg;
            grid_map::PolygonRosConverter::toMessage(pose_footprint, poly_msg);
            poly_debug_pub_.publish(poly_msg);
        }

        if (static_map_fused_.exists("occupancy"))
        {
            grid_map::Matrix& static_map_data = static_map_fused_["occupancy"];

            for (grid_map::PolygonIterator poly_iterator(static_map_fused_, pose_footprint); !poly_iterator.isPastEnd(); ++poly_iterator) {
                const grid_map::Index index(*poly_iterator);

                static_map_data(index(0), index(1)) = 0.0;
            }
        }
        else
        {
            ROS_WARN("no occupancy layer");
        }
    }else{
        ROS_WARN("Robot pose not available in worldmodel callback, not clearing footprint.");
    }


   this->publishFusedNavGrid();*/
}

void MapCombiner::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    robot_pose_ = msg;
    //this->segmentObstacleAt(grid_map::Position(msg->pose.position.x, msg->pose.position.y), 2.0);
}

void MapCombiner::collisionPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    this->segmentObstacleAt(msg->pose, 0.8);
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
    ROS_ERROR("elevation %f",robot_elevation);

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
                                               cv::Point( erosion_sie, erosion_size ) );

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

    if(!static_cut.exists("occupancy") || !local_elevation_map_.exists("elevation") || !static_cut.exists("occupancy_inflated"))
    {
        ROS_WARN("no occupancy /elevation /occupancy_inflated");
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


            if ( std::abs( robot_elevation - elev_data(elev_index(0), elev_index(1)) ) > p_obstacle_neg_diff_threshold_ ){
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


void MapCombiner::segmentObstacleAt(const geometry_msgs::Pose& pose, const double search_area_edge_length)
{
    ROS_INFO("segmentObstacleAt");
    if (!local_elevation_map_.exists("elevation")){
        ROS_WARN("Elevation map not available, skipping obstacle segmentation!");
        return;
    }

    if (!robot_pose_.get()){
        ROS_WARN("Cannot retrieve robot pose in obstacle segmentation, aborting.");
        return;
    }

    grid_map::Position pos(pose.position.x, pose.position.y);


    float robot_elevation = robot_pose_->pose.position.z + p_pose_height_offset_;

    bool submap_create_success;
    grid_map::Length local_length(search_area_edge_length, search_area_edge_length);
    grid_map::GridMap static_cut = this->static_map_retrieved_.getSubmap(pos, local_length, submap_create_success);

    if (!submap_create_success){
        ROS_ERROR("Failed to create submap, skipping obstacle segmentation!");
        return;
    }

    cv::Mat static_map_cv;
    grid_map::GridMapRosConverter::toCvImage(static_cut, "occupancy_inflated", static_map_cv);

    //ROS_WARN("type: %d", static_map_cv.type());
    //debug_img_provider_->addDebugImage(static_cut_mat);

    cv::Mat static_map_cv_gray;
    cv::cvtColor(static_map_cv, static_map_cv_gray, CV_BGRA2GRAY);

    flood_debug_img_provider_->addDebugImage(static_map_cv);

    cv::Point2d seed(static_map_cv_gray.size().width/2, static_map_cv_gray.size().height/2);


    cv::Mat mask = cv::Mat::zeros(static_map_cv_gray.rows + 2, static_map_cv_gray.cols + 2, CV_8U);
    cv::floodFill(static_map_cv_gray, mask, seed, 255, 0, cv::Scalar(), cv::Scalar(),  4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

    cv_bridge::CvImage mask_cut;
    mask_cut.encoding = sensor_msgs::image_encodings::MONO8;

    cv::Rect roi (1, 1, static_map_cv_gray.rows, static_map_cv_gray.cols);
    mask_cut.image = mask(roi);
    flood_debug_img_provider_->addDebugImage(mask_cut.image);


    sensor_msgs::ImagePtr img = mask_cut.toImageMsg();

    if (!grid_map::GridMapRosConverter::addLayerFromImage(*img.get(), "obstacle_check_mask", static_cut)){
        ROS_ERROR("Failed to add image layer, skipping obstacle segmentation!");
        return;
    }



    //grid_map::GridMap elevation_cut = this->static_map_retrieved_.getSubmap(pos, local_length, submap_create_success);

    grid_map::Matrix& elev_data   = local_elevation_map_["elevation"];
    grid_map::Matrix& static_cut_mask = static_cut["obstacle_check_mask"];

    grid_map::Polygon footprint_poly = grid_map_polygon_tools::getTransformedPoly(this->footprint_poly_, this->robot_pose_->pose);

    for (grid_map::PolygonIterator poly_iterator(static_cut, footprint_poly); !poly_iterator.isPastEnd(); ++poly_iterator) {

        const grid_map::Index index(*poly_iterator);

        static_cut_mask(index(0), index(1)) = 0.0;
    }


    //std::vector<grid_map::Position> obstacle_points;
    std::vector<cv::Point2f> obstacle_points;
    std::vector<double> obstacle_elevations;

    //@TODO This can be made faster as described in https://github.com/ethz-asl/grid_map/issues/53
    for (grid_map::GridMapIterator iterator(static_cut); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);

        if (static_cut_mask(index(0), index(1)) > 0.5){

            grid_map::Position position;
            static_cut.getPosition(index, position);

            grid_map::Index elev_index;
            local_elevation_map_.getIndex(position, elev_index);

            //std::cout << "re: " << robot_elevation << " elev: " << elev_data(index(0), index(1)) << "\n";

            //std::cout << "re: " << robot_elevation << " elev: " << static_cut_inflated_data(index(0), index(1)) << "\n";

            //if (static_data(index(0), index(1)) < 0.001){


            if ( std::abs( robot_elevation - elev_data(elev_index(0), elev_index(1)) ) > (p_obstacle_neg_diff_threshold_ - 0.05) ){
                obstacle_points.push_back(cv::Point2f(position.x(), position.y()));
                obstacle_elevations.push_back(elev_data(elev_index(0), elev_index(1)));
            }
        }
    }



    if (obstacle_points.size() > 3){

        double avg_elevation = 0.0;

        for (size_t i = 0; i < obstacle_elevations.size(); ++i){
            avg_elevation += obstacle_elevations[i];
        }

        avg_elevation = avg_elevation / obstacle_elevations.size();

        cv::RotatedRect rect = cv::minAreaRect(obstacle_points);


        if (obstacle_poly_pub_.getNumSubscribers() > 0){
            geometry_msgs::PolygonStamped poly;
            poly.header.frame_id = "world";
            poly.header.stamp = ros::Time::now();

            cv::Point2f points [4];

            rect.points(points);
            poly.polygon.points.resize(4);
            for (size_t i = 0; i < 4; ++i){
                poly.polygon.points[i].x = points[i].x;
                poly.polygon.points[i].y = points[i].y;
            }

            obstacle_poly_pub_.publish(poly);
        }

        if (obstacle_marker_pub_.getNumSubscribers() > 0){

            // Negative
            if (avg_elevation < 0.0){
                obstacle_marker_box_.color.a = 0.5;
                obstacle_marker_box_.color.r = 1.0;
                obstacle_marker_box_.color.b = 0.0;
                obstacle_marker_box_.color.g = 0.0;

                obstacle_marker_text_.text = "Type: Negative Obst.\nRecommendation: Find other Pathway";

                // Positive
            }else if (avg_elevation < 0.2){
                obstacle_marker_box_.color.a = 0.5;
                obstacle_marker_box_.color.r = 0.0;
                obstacle_marker_box_.color.b = 1.0;
                obstacle_marker_box_.color.g = 0.0;

                obstacle_marker_text_.text = "Type: Positive Obst.\nRecommendation: Find other Pathway";

                // Suspended
            }else{
                obstacle_marker_box_.color.a = 0.5;
                obstacle_marker_box_.color.r = 0.0;
                obstacle_marker_box_.color.b = 1.0;
                obstacle_marker_box_.color.g = 0.0;

                obstacle_marker_text_.text = "Type: Suspended Obst.\nRecommendation: Find other Pathway";

            }



            obstacle_marker_box_.header.stamp = ros::Time::now();
            obstacle_marker_box_.pose.position.x = rect.center.x;
            obstacle_marker_box_.pose.position.y = rect.center.y;
            //@TODO: Will not work when away from 0 plane
            obstacle_marker_box_.pose.position.z = avg_elevation *0.5;

            obstacle_marker_box_.scale.x = rect.size.width;
            obstacle_marker_box_.scale.y = rect.size.height;
            obstacle_marker_box_.scale.y = avg_elevation;
            obstacle_marker_box_.scale.z = 0.1;


            obstacle_marker_text_.header = obstacle_marker_box_.header;
            obstacle_marker_text_.pose = obstacle_marker_box_.pose;
            obstacle_marker_text_.pose.position.z += 0.3;
            obstacle_marker_text_.color = obstacle_marker_box_.color;
            obstacle_marker_text_.color.a = 0.95;

            if(rect.size.width < rect.size.height){
                //printf("Angle along longer side: %7.2f\n", calculatedRect.angle+180);
                Eigen::Quaterniond  quat (Eigen::AngleAxisd((rect.angle+90.0) * (M_PI/180.0) , Eigen::Vector3d::UnitZ()));
                tf::quaternionEigenToMsg(quat, obstacle_marker_box_.pose.orientation);
            }else{
                //printf("Angle along longer side: %7.2f\n", calculatedRect.angle+90);
                Eigen::Quaterniond  quat (Eigen::AngleAxisd((rect.angle+180.0) * (M_PI/180.0), Eigen::Vector3d::UnitZ()));
                tf::quaternionEigenToMsg(quat, obstacle_marker_box_.pose.orientation);
            }

            visualization_msgs::MarkerArray array;
            array.markers.push_back(obstacle_marker_box_);
            array.markers.push_back(obstacle_marker_text_);

            obstacle_marker_pub_.publish(array);


            if (p_publish_percept_){

                hector_worldmodel_msgs::PosePercept pose_percept;

                pose_percept.header.frame_id = "world";
                pose_percept.header.stamp = ros::Time::now();
                pose_percept.info.class_id = "obstacle";
                pose_percept.info.object_support = 1.0;
                pose_percept.pose.pose = pose;

                pose_percept_publisher_.publish(pose_percept);
            }
        }




    }

    /*
  //Perform PCA analysis
  cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
  //Store the center of the object
  cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                             static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  //Store the eigenvalues and eigenvectors
  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i)
  {
    eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }
  */


    flood_debug_img_provider_->publishDebugImage();
}



void MapCombiner::publishFusedNavGrid()
{

    if(!static_map_fused_.exists("occupancy"))
    {
        ROS_WARN("no occupancy layer");
        return;
    }
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
    p_obstacle_neg_diff_threshold_ = config.elev_diff_negative_threshold;
    p_obstacle_pos_diff_threshold_ = config.elev_diff_positive_threshold;
    p_pose_height_offset_      = config.pose_height_offset;
    p_large_obstacle_radius_   = config.large_obstacle_radius;
    p_small_obstacle_radius_   = config.small_obstacle_radius;

    p_publish_percept_         = config.publish_percept;
    p_fuse_elevation_map_      = config.fuse_elevation_map;

    p_elevation_map_topic      = config.elevation_map_topic;
    p_base_map_topic           = config.base_map_topic;

    grid_map_polygon_tools::setFootprintPoly(config.footprint_x, config.footprint_y, this->footprint_poly_);
    local_elevation_map_sub_ = pnh.subscribe(p_elevation_map_topic, 1, &MapCombiner::localElevationMapCallback, this);
    static_map_sub_ = pnh.subscribe(p_base_map_topic, 1, &MapCombiner::staticMapCallback, this);


    //ROS_INFO("MapCombiner params set: obstacle thresh: %f height offset: %f", p_obstacle_diff_threshold_,  p_pose_height_offset_);
    ROS_INFO("MapCombiner params set: \n elevation_map_topic: %s \n base_map_topic offset: %s \n fusing elevation map: %i", p_elevation_map_topic.c_str(),  p_base_map_topic.c_str(),p_fuse_elevation_map_);
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
