/// HEADER
#include "planner_node.h"

/// PROJECT
#include <cslibs_path_planning/common/CollisionGridMap2d.h>
#include <cslibs_path_planning/common/RotatedGridMap2d.h>
#include <cslibs_path_planning/common/Bresenham2d.h>
#include <cslibs_utils/Stopwatch.h>

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/publisher.h>
#include <string.h>

using namespace lib_path;

Planner::Planner()
    : nh_priv("~"),
      is_cost_map_(false),
      server_(nh, "plan_path", boost::bind(&Planner::execute, this, _1), false),
      map_info(NULL), map_rotation_yaw_(0.0), thread_running(false)
{
    std::string target_topic = "/goal";
    nh_priv.param("target_topic", target_topic, target_topic);

    //    goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //            (target_topic, 2, boost::bind(&Planner::updateGoalCallback, this, _1));

    nh_priv.param("use_map_topic", use_map_topic_, false);
    nh_priv.param("use_map_service", use_map_service_, !use_map_topic_);

    if(use_map_topic_) {
        std::string map_topic = nh_priv.param("map_topic", std::string("map"));
        map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
                (map_topic, 1, boost::bind(&Planner::updateMapCallback, this, _1));

        ROS_INFO_STREAM("using map topic " << map_topic);

    } else {
        std::string map_service = nh_priv.param("map_service", std::string("static_map"));
        map_service_client = nh_priv.serviceClient<nav_msgs::GetMap> (map_service);

        ROS_INFO_STREAM("using map service " << map_service);
    }

    nh_priv.param("preprocess", pre_process_, true);
    nh_priv.param("postprocess", post_process_, true);

    nh_priv.param("grow_obstacles", grow_obstacles_, 0.0);

    nh_priv.param("use_cost_map", use_cost_map_, false);
    if(use_cost_map_ && !pre_process_) {
        use_cost_map_service_ = true;
        std::string costmap_service = nh_priv.param("cost_map_service", std::string("dynamic_map/cost"));
        cost_map_service_client = nh_priv.serviceClient<nav_msgs::GetMap> (costmap_service);

        ROS_INFO_STREAM("using cost map service " << costmap_service);

    } else {
        use_cost_map_service_ = false;
    }

    nh_priv.param("optimization/optimize_cost", post_process_optimize_cost_, false);
    if(post_process_optimize_cost_) {
        nh_priv.param("optimization/publish_gradient", publish_gradient_, false);

        nh_priv.param("optimization/weight/data", cost_optimization_weight_data, 0.9);
        nh_priv.param("optimization/weight/smooth", cost_optimization_weight_smooth, 0.3);
        nh_priv.param("optimization/weight/cost", cost_optimization_weight_cost, 0.75);
        nh_priv.param("optimization/tolerance", cost_optimization_tolerance, 1e-5);
    }

    nh_priv.param("use_cloud", use_cloud_, true);
    nh_priv.param("use_scan_front", use_scan_front_, false);
    nh_priv.param("use_scan_back", use_scan_back_, false);

    if(use_cloud_) {
        ROS_INFO_STREAM("subscribing to obstacle cloud topic " << nh.resolveName(std::string("obstacle_cloud")));
        sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("obstacle_cloud", 0, boost::bind(&Planner::cloudCallback, this, _1));
        //        sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/obstacle_cloud", 0, boost::bind(&Planner::cloudCallback, this, _1));
    }
    if(use_scan_front_) {
        sub_front = nh_priv.subscribe<sensor_msgs::LaserScan>("scan/front/filtered", 0, boost::bind(&Planner::laserCallback, this, _1, true));
    }
    if(use_scan_back_) {
        sub_back = nh_priv.subscribe<sensor_msgs::LaserScan>("scan/back/filtered", 0, boost::bind(&Planner::laserCallback, this, _1, false));
    }


    nh_priv.param("use_collision_gridmap", use_collision_gridmap_, false);

    viz_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    viz_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
    cost_pub = nh_priv.advertise<nav_msgs::OccupancyGrid>("cost", 1, true);
    map_pub = nh_priv.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    std::string default_world_frame = nh.param("gerona/world_frame", std::string("map"));
    nh_priv.param("world_frame", world_frame_, default_world_frame);

    std::string default_robot_frame = nh.param("gerona/robot_frame", std::string("base_link"));
    nh_priv.param("robot_frame", robot_frame_, default_robot_frame);


    nh_priv.param("size/forward", size_forward, 0.4);
    nh_priv.param("size/backward", size_backward, -0.6);
    nh_priv.param("size/width", size_width, 0.5);

    path_publisher_ = nh.advertise<path_msgs::PathSequence> ("path", 10);
    raw_path_publisher_ = nh.advertise<nav_msgs::Path> ("path_raw", 10);

    server_.registerPreemptCallback(boost::bind(&Planner::preempt, this));
    server_.start();
}

Planner::~Planner()
{

}

void Planner::preempt()
{
    ROS_WARN("preempting!!");

    thread_mutex.lock();
    bool running = thread_running;
    thread_mutex.unlock();
    if(running) {
        ROS_WARN_STREAM("preempting path planner");
        thread_->interrupt();
        thread_->join();
    }

    server_.setPreempted();

    ROS_WARN_STREAM("done preempting path planner");
}

void Planner::feedback(int status)
{
    if(server_.isActive()) {
        path_msgs::PlanPathFeedback f;
        f.status = status;
        server_.publishFeedback(f);
    }
}

void Planner::updateMapCallback (const nav_msgs::OccupancyGridConstPtr &map)
{
    pending_map = map;
}

void Planner::updateMap (const nav_msgs::OccupancyGrid &map, bool is_cost_map)
{
    boost::lock_guard<boost::mutex> lock(map_mutex);

    is_cost_map_ = is_cost_map;

    unsigned w = map.info.width;
    unsigned h = map.info.height;

    bool replace = map_info == NULL ||
            map_info->getWidth() != w ||
            map_info->getHeight() != h;

    if(replace){
        if(map_info != NULL) {
            delete map_info;
        }


        if(use_collision_gridmap_) {
            map_info = new lib_path::CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width);
        } else {
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(map.info.origin.orientation, orientation);
            if(orientation != tf::Quaternion(0., 0., 0., 1.0)) {
                map_rotation_yaw_ = tf::getYaw(orientation);
                map_info = new lib_path::RotatedGridMap2d(map.info.width, map.info.height, map_rotation_yaw_, map.info.resolution);
            } else {
                map_info = new lib_path::SimpleGridMap2d(map.info.width, map.info.height, map.info.resolution);
            }
        }
    }

    std::vector<uint8_t> data(w*h);
    int i = 0;

    if(is_cost_map) {
        map_info->setLowerThreshold(253);
        map_info->setUpperThreshold(254);
        map_info->setNoInformationValue(255);

        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            uint8_t val = *it;
            data[i++] = val;
        }


    } else {
        bool use_unknown;
        nh_priv.param("use_unknown_cells", use_unknown, true);

        if(use_unknown) {
            /// Map data
            /// -1: unknown -> 0
            /// 0:100 probabilities -> 1 - 100
            for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
                data[i++] = std::min(100, *it + 1);
            }

        } else {
            /// Map data
            /// -1: unknown -> -1
            /// 0:100 probabilities -> 0 - 100
            for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
                data[i++] = *it;
            }
        }

        map_info->setLowerThreshold(50);
        map_info->setUpperThreshold(70);
        map_info->setNoInformationValue(-1);
    }

    map_info->set(data, w, h);
    map_info->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));

    cost_map.header = map.header;
    cost_map.info = map.info;
}

void Planner::visualizeOutline(const geometry_msgs::Pose& at, int id, const std::string &frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "planning/outline";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.75;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;


    tf::Pose fl_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, size_width / 2.0, 0.0));
    tf::Pose fr_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, -size_width / 2.0, 0.0));
    tf::Pose bl_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, size_width / 2.0, 0.0));
    tf::Pose br_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, -size_width / 2.0, 0.0));

    tf::Transform transform;
    tf::poseMsgToTF(at, transform);

    tf::Pose fl_ = transform * fl_base;
    tf::Pose fr_ = transform * fr_base;
    tf::Pose bl_ = transform * bl_base;
    tf::Pose br_ = transform * br_base;

    geometry_msgs::Point fl, fr, bl, br;
    fl.x = fl_.getOrigin().x();
    fl.y = fl_.getOrigin().y();
    fr.x = fr_.getOrigin().x();
    fr.y = fr_.getOrigin().y();
    bl.x = bl_.getOrigin().x();
    bl.y = bl_.getOrigin().y();
    br.x = br_.getOrigin().x();
    br.y = br_.getOrigin().y();


    marker.points.push_back(fl);
    marker.points.push_back(fr);
    marker.points.push_back(fr);
    marker.points.push_back(br);
    marker.points.push_back(br);
    marker.points.push_back(bl);
    marker.points.push_back(bl);
    marker.points.push_back(fl);

    viz_pub.publish(marker);
}

void Planner::visualizePath(const path_msgs::PathSequence& path, int id, double alpha)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "planning/steps";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = alpha;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    tf::Pose fl_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, size_width / 4.0, 0.0));
    tf::Pose fr_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, -size_width / 4.0, 0.0));
    tf::Pose bl_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, size_width / 2.0, 0.0));
    tf::Pose br_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, -size_width / 2.0, 0.0));

    for(const path_msgs::DirectionalPath& dpath : path.paths) {
        for (const geometry_msgs::PoseStamped& spose : dpath.poses) {

            const geometry_msgs::Pose& pose = spose.pose;
            tf::Transform transform;
            tf::poseMsgToTF(pose, transform);

            tf::Pose fl_ = transform * fl_base;
            tf::Pose fr_ = transform * fr_base;
            tf::Pose bl_ = transform * bl_base;
            tf::Pose br_ = transform * br_base;

            geometry_msgs::Point fl, fr, bl, br;
            fl.x = fl_.getOrigin().x();
            fl.y = fl_.getOrigin().y();
            fr.x = fr_.getOrigin().x();
            fr.y = fr_.getOrigin().y();
            bl.x = bl_.getOrigin().x();
            bl.y = bl_.getOrigin().y();
            br.x = br_.getOrigin().x();
            br.y = br_.getOrigin().y();


            marker.points.push_back(fl);
            marker.points.push_back(fr);
            marker.points.push_back(fr);
            marker.points.push_back(br);
            marker.points.push_back(br);
            marker.points.push_back(bl);
            marker.points.push_back(bl);
            marker.points.push_back(fl);
        }
    }
    viz_pub.publish(marker);

    marker.ns = "planning/lines";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.clear();
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for(const path_msgs::DirectionalPath& dpath : path.paths) {
        for (const geometry_msgs::PoseStamped& spose : dpath.poses) {
            const geometry_msgs::Pose& pose = spose.pose;
            geometry_msgs::Point pt;
            pt.x = pose.position.x;
            pt.y = pose.position.y;
            marker.points.push_back(pt);
        }
    }

    if(marker.points.size() < 30000) {
        viz_pub.publish(marker);
    } else {
        ROS_WARN_STREAM("don't visualze path with " << marker.points.size() << " to avoid RVIZ crashes.") ;
    }
}

void Planner::updateGoalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
    ROS_INFO("planner: got goal");

    path_msgs::PlanPathGoal request;
    request.use_start = false;
    request.goal.type = path_msgs::Goal::GOAL_TYPE_POSE;
    request.goal.pose = *goal;
    request.start = lookupPose();

    findPath(request);
}

void Planner::execute(const path_msgs::PlanPathGoalConstPtr &goal)
{
    Stopwatch sw_global;

    Stopwatch sw;
    sw.reset();
    path_msgs::PathSequence path = findPath(*goal);
    ROS_DEBUG_STREAM("findPath took " << sw.msElapsed() << "ms");

    if(path.paths.empty()) {
        feedback(path_msgs::PlanPathFeedback::STATUS_PLANNING_FAILED);

        path_msgs::PlanPathResult fail;
        server_.setAborted(fail, "no path found");

    } else {
        feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

        path_msgs::PlanPathResult success;
        success.path = path;
        server_.setSucceeded(success);
    }
    ROS_INFO_STREAM("path planning took " << sw_global.msElapsed() << "ms");
}

path_msgs::PathSequence Planner::findPath(const path_msgs::PlanPathGoal& request)
{
    Stopwatch sw;
    if(use_map_topic_ && pending_map) {
        updateMap(*pending_map, false);

    } else if(use_cost_map_service_) {
        sw.reset();
        nav_msgs::GetMap map_service;
        if(!cost_map_service_client.exists()) {
            cost_map_service_client.waitForExistence();
        }
        if(cost_map_service_client.call(map_service)) {
            cost_map = map_service.response.map;
            updateMap(map_service.response.map, true);
        } else {
            ROS_ERROR("call to costmap service failed");
        }
        ROS_INFO_STREAM("cost map service lookup took " << sw.msElapsed() << "ms");

    } else if(use_map_service_) {
        sw.reset();
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            updateMap(map_service.response.map, false);
        } else {
            ROS_ERROR("map service lookup failed");
            return path_msgs::PathSequence();
        }
        ROS_INFO_STREAM("map service lookup took " << sw.msElapsed() << "ms");

    } else {
        nav_msgs::OccupancyGrid empty_map;
        empty_map.header.frame_id = world_frame_;
        empty_map.header.stamp = ros::Time::now();
        empty_map.info.resolution = 0.05;

        double w = 0.0;
        double h = 0.0;

        geometry_msgs::PoseStamped start = request.use_start ? request.start : lookupPose();

        switch(request.goal.type) {
        case path_msgs::Goal::GOAL_TYPE_POSE: {
            lib_path::Pose2d from_map;
            from_map.x  = start.pose.position.x / empty_map.info.resolution;
            from_map.y  = start.pose.position.y / empty_map.info.resolution;
            from_map.theta  = tf::getYaw(start.pose.orientation);

            lib_path::Pose2d to_map;
            to_map.x  = request.goal.pose.pose.position.x / empty_map.info.resolution;
            to_map.y  = request.goal.pose.pose.position.y / empty_map.info.resolution;
            to_map.theta  = tf::getYaw(request.goal.pose.pose.orientation);

            int min_x = std::min(from_map.x, to_map.x);
            int max_x = std::max(from_map.x, to_map.x);

            int min_y = std::min(from_map.y, to_map.y);
            int max_y = std::max(from_map.y, to_map.y);

            int margin = 5.0 / empty_map.info.resolution;

            empty_map.info.width = max_x - min_x + 2 * margin;
            empty_map.info.height = max_y - min_y + 2 * margin;

            w = empty_map.info.width * empty_map.info.resolution;
            h = empty_map.info.height * empty_map.info.resolution;

            empty_map.info.origin.position.x = (to_map.x + from_map.x) * empty_map.info.resolution / 2.0 - w/2.0;
            empty_map.info.origin.position.y = (to_map.y + from_map.y) * empty_map.info.resolution / 2.0 - h/2.0;
            break;
        }
        case path_msgs::Goal::GOAL_TYPE_MAP:
            w = 80.0;
            h = 80.0;
            empty_map.info.width = w / empty_map.info.resolution;
            empty_map.info.height = h / empty_map.info.resolution;
            empty_map.info.origin.position.x = start.pose.position.x - w/2.0;
            empty_map.info.origin.position.y = start.pose.position.y - h/2.0;

            break;

        default:
            ROS_FATAL_STREAM("requested goal type " << request.goal.type << " is unknown.");
            return empty();
        }

        empty_map.info.origin.orientation.w = 1.0;
        empty_map.data.resize(empty_map.info.width * empty_map.info.height, 0);

        ROS_DEBUG_STREAM("no map to use, generate empty map");
        updateMap(empty_map, false);
    }

    if(map_info == NULL) {
        ROS_ERROR("request for path planning, but no map there yet...");
        return path_msgs::PathSequence();
    }
    //    cv::Mat map_raw(map_info->getHeight(), map_info->getWidth(), CV_8UC1, map_info->getData());
    //    cv::imshow("map_raw", map_raw);
    //    cv::waitKey(100);



    sw.reset();
    preprocess(request);
    ROS_DEBUG_STREAM("preprocessing took " << sw.msElapsed() << "ms");


    sw.reset();
    path_msgs::PathSequence path_raw = doPlan(request);
    ROS_DEBUG_STREAM("planning took " << sw.msElapsed() << "ms");

    path_msgs::PathSequence path;
    if(post_process_ && !path_raw.paths.empty()) {
        sw.reset();
        path = postprocess(path_raw);
        ROS_DEBUG_STREAM("postprocessing took " << sw.msElapsed() << "ms");
    } else {
        path = path_raw;
    }

    sw.reset();
    publish(path, path_raw);
    ROS_DEBUG_STREAM("publish took " << sw.msElapsed() << "ms");
    return path;
}


void Planner::preprocess(const path_msgs::PlanPathGoal& request)
{
    boost::lock_guard<boost::mutex> lock(map_mutex);

    feedback(path_msgs::PlanPathFeedback::STATUS_PRE_PROCESSING);

    if(use_cloud_ && !cloud_.data.empty()) {
        integratePointCloud(cloud_);
    }
    if(use_scan_front_ && !scan_front.ranges.empty()) {
        integrateLaserScan(scan_front);
    }
    if(use_scan_back_ && !scan_back.ranges.empty()) {
        integrateLaserScan(scan_back);
    }

    if(grow_obstacles_ != 0.0) {
        growObstacles(request, request.options.grow_obstacles ? request.options.obstacle_growth_radius : grow_obstacles_);
    }

    if(map_pub.getNumSubscribers() > 0) {
        nav_msgs::OccupancyGrid map_viz;
        map_viz.header.stamp = ros::Time::now();
        map_viz.header.frame_id = world_frame_;
        map_viz.info.origin.position.x = map_info->getOrigin().x;
        map_viz.info.origin.position.y = map_info->getOrigin().y;
        map_viz.info.resolution = map_info->getResolution();
        map_viz.info.width = map_info->getWidth();
        map_viz.info.height = map_info->getHeight();

        std::size_t n = map_info->getWidth() * map_info->getHeight() * sizeof(unsigned char);
        map_viz.data.resize(n);
        int8_t* data = map_viz.data.data();
        //std::memcpy(&map_viz.data.at(0), map_info->getData(), n);
        for(int y = 0, h = map_info->getHeight(); y < h; ++y) {
            for(int x = 0, w = map_info->getWidth(); x < w; ++x, ++data) {

                *data = map_info->isFree(x,y) ? 255 : 0;
            }
        }
        map_pub.publish(map_viz);
    }

    if(pre_process_) {
        // cost
        int h = map_info->getHeight();
        int w = map_info->getWidth();
        cv::Mat map(h, w, CV_8UC1, map_info->getData());
        cost_map.data.resize(h*w);
        cv::Mat costmap(h, w, CV_8UC1, cost_map.data.data());
        map.copyTo(costmap);

        //    cv::Mat unknown_mask;
        //    cv::inRange(map, 0, 0, unknown_mask);
        //    costmap.setTo(0, unknown_mask);
        //    cv::threshold(costmap, costmap, 50, 255, cv::THRESH_BINARY);

        costmap = 100 - costmap;

        cv::Mat distance;
        cv::distanceTransform(costmap, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

        double scale_  = 100.0;
        double max_distance_meters_ = 2.5;
        double factor = (scale_ * map_info->getResolution() / max_distance_meters_);
        distance.convertTo(costmap, CV_8UC1, factor);

        //    cv::threshold(costmap, costmap, 98, 98, CV_THRESH_TRUNC);
        costmap = cv::max(0, 98 - costmap);
        //    costmap.setTo(50, unknown_mask);

        cv::imwrite("costmap.png", costmap);

        cost_pub.publish(cost_map);
    }
}

path_msgs::PathSequence Planner::postprocess(const path_msgs::PathSequence& path)
{
    boost::lock_guard<boost::mutex> lock(map_mutex);

    Stopwatch sw;

    ROS_DEBUG("postprocessing");

    feedback(path_msgs::PlanPathFeedback::STATUS_POST_PROCESSING);

    path_msgs::PathSequence working_copy = path;

    //    path_msgs::PathSequence simplified_path = simplifyPath(path);
    //    ROS_INFO_STREAM("simplifying took " << sw.msElapsed() << "ms");

    if(post_process_optimize_cost_) {
        sw.restart();
        working_copy = optimizePathCost(working_copy);
        ROS_DEBUG_STREAM("optimizing cost took " << sw.msElapsed() << "ms");
    }

    sw.restart();
    path_msgs::PathSequence interpolated_path = interpolatePath(working_copy, 0.5);
    ROS_DEBUG_STREAM("interpolation took " << sw.msElapsed() << "ms");


    sw.restart();
    path_msgs::PathSequence smoothed_path = smoothPath(interpolated_path, 0.6, 0.15);
    ROS_DEBUG_STREAM("smoothing took " << sw.msElapsed() << "ms");


    sw.restart();
    path_msgs::PathSequence final_interpolated_path = interpolatePath(smoothed_path, 0.1);
    ROS_DEBUG_STREAM("final interpolation took " << sw.msElapsed() << "ms");


    sw.restart();
    path_msgs::PathSequence final_smoothed_path = smoothPath(final_interpolated_path, 2.0, 0.4);
    ROS_DEBUG_STREAM("final smoothing took " << sw.msElapsed() << "ms");

    return final_smoothed_path;
}


geometry_msgs::PoseStamped Planner::lookupPose()
{
    geometry_msgs::PoseStamped own_pose;
    own_pose.header.frame_id = world_frame_;
    own_pose.header.stamp = ros::Time::now();
    tf::StampedTransform trafo = lookupTransform(own_pose.header.frame_id, robot_frame_, own_pose.header.stamp);
    tf::poseTFToMsg(trafo, own_pose.pose);

    return own_pose;
}

tf::StampedTransform Planner::lookupTransform(const std::string& from, const std::string& to, const ros::Time& stamp)
{
    tf::StampedTransform trafo;
    if(tfl.waitForTransform(from, to, stamp, ros::Duration(1.0))) {
        tfl.lookupTransform(from, to, stamp, trafo);
    } else {
        ROS_WARN("cannot lookup own pose, using last estimate");
        // this can throw...
        if(!tfl.canTransform(from, to, ros::Time(0))) {
            tfl.waitForTransform(from, to, ros::Time(0), ros::Duration(0.01));
        }
        if(!tfl.canTransform(from, to, ros::Time(0))) {
            ROS_FATAL_STREAM("cannot get latest transform from " << from << " to " << to);
        }
        tfl.lookupTransform(from, to, ros::Time(0), trafo);
    }
    return trafo;
}


path_msgs::PathSequence Planner::doPlan(const path_msgs::PlanPathGoal &request)
{
    if(!supportsGoalType(request.goal.type)) {
        ROS_FATAL_STREAM("requested goal type " << request.goal.type << " is not supported.");
        return empty();
    }

    ROS_DEBUG("starting search");

    feedback(path_msgs::PlanPathFeedback::STATUS_PLANNING);

    boost::thread worker(boost::bind(&Planner::planThreaded, this, request));
    ros::Rate spin(10);
    ros::Time start_time = ros::Time::now();
    ros::Duration max_search_time(std::max(request.options.max_search_duration, 40.f));

    while(ros::ok()) {
        ROS_INFO_STREAM_THROTTLE(2, "still planning");
        bool timeout = start_time + max_search_time < ros::Time::now();
        if(timeout){
            ROS_ERROR("search timed out");
        }
        if(server_.isPreemptRequested() || timeout) {
            ROS_INFO_STREAM("preempted path planner");
            return path_msgs::PathSequence();

        } else {
            bool running;
            thread_mutex.lock();
            running = thread_running;
            thread_ = &worker;
            thread_mutex.unlock();

            if(!running) {
                worker.join();
                break;
            }
        }
        spin.sleep();
    }

    ROS_DEBUG_STREAM("sub-planner done or aborted");

    thread_mutex.lock();
    path_msgs::PathSequence path = thread_result;
    thread_mutex.unlock();

    return path;
}

void Planner::planThreaded(const path_msgs::PlanPathGoal &goal)
{
    thread_mutex.lock();
    thread_running = true;
    thread_mutex.unlock();

    path_msgs::PathSequence path;
    {
        boost::lock_guard<boost::mutex> lock(map_mutex);
        path = planImpl(goal);
    }

    thread_mutex.lock();
    thread_result = path;
    thread_running = false;
    thread_mutex.unlock();
}

path_msgs::PathSequence Planner::planImpl(const path_msgs::PlanPathGoal &request)
{
    geometry_msgs::PoseStamped start = request.use_start ? request.start : lookupPose();
    lib_path::Pose2d from_world, from_map;
    transformPose(start, from_world, from_map);

    switch(request.goal.type) {
    case path_msgs::Goal::GOAL_TYPE_POSE: {
        lib_path::Pose2d to_world, to_map;
        transformPose(request.goal.pose, to_world, to_map);

        return plan(request, from_world, to_world, from_map, to_map);
    }
    case path_msgs::Goal::GOAL_TYPE_MAP:
        return planWithoutTargetPose(request, from_world, from_map);

    default:
        ROS_FATAL_STREAM("requested goal type " << request.goal.type << " is unknown.");
        return empty();
    }
}

path_msgs::PathSequence Planner::planWithoutTargetPose(const path_msgs::PlanPathGoal &request,
                                                       const Pose2d &from_world, const Pose2d &from_map)
{
    ROS_FATAL_STREAM("requested goal type " << request.goal.type << " is not implemented.");
    return empty();
}

void Planner::transformPose(const geometry_msgs::PoseStamped& pose, lib_path::Pose2d& world, lib_path::Pose2d& map)
{
    ROS_ASSERT(map_info != nullptr);

    world.x = pose.pose.position.x;
    world.y = pose.pose.position.y;
    world.theta = tf::getYaw(pose.pose.orientation);

    if(std::isnan(world.theta)) {
        world.theta = 0.0;
    }

    visualizeOutline(pose.pose, 0, world_frame_);

    unsigned fx, fy;
    map_info->point2cell(world.x, world.y, fx, fy);
    map.x = (int) fx;
    map.y = (int) fy;
    map.theta = world.theta - map_rotation_yaw_;

    if(std::isnan(map.theta)) {
        map.theta = 0.0;
    }
}

void Planner::subdividePath(path_msgs::DirectionalPath &result, geometry_msgs::PoseStamped low, geometry_msgs::PoseStamped up, double max_distance) {
    double dx = low.pose.position.x - up.pose.position.x;
    double dy = low.pose.position.y - up.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if(distance > max_distance) {
        // split half way between the lower and the upper node
        geometry_msgs::PoseStamped halfway(low);
        halfway.pose.position.x += (up.pose.position.x - low.pose.position.x) / 2.0;
        halfway.pose.position.y += (up.pose.position.y - low.pose.position.y) / 2.0;

        double up_y = tf::getYaw(up.pose.orientation);
        double low_y = tf::getYaw(low.pose.orientation);

        halfway.pose.orientation = tf::createQuaternionMsgFromYaw((up_y + low_y) / 2.0);

        // first recursive descent in lower part
        subdividePath(result, low, halfway, max_distance);
        // then add the half way point
        result.poses.push_back(halfway);
        // then descent in upper part
        subdividePath(result, halfway, up, max_distance);
    }
}

namespace {
bool isFree(SimpleGridMap2d* map_ptr, const nav_msgs::OccupancyGrid& costmap,
            const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
    lib_path::Bresenham2d bresenham;

    unsigned int fx, fy, tx, ty;
    map_ptr->point2cell(from.x, from.y, fx, fy);
    map_ptr->point2cell(to.x, to.y, tx, ty);
    bresenham.setGrid(map_ptr, fx, fy, tx, ty);

    std::size_t w = costmap.info.width;

    unsigned x,y;
    while(bresenham.next()) {
        bresenham.coordinates(x,y);
        if(!map_ptr->isFree(x,y)) {
            return false;
        }
        std::size_t idx = y * w + x;
        if(costmap.data[idx] > 5) {
            return false;
        }
    }

    return true;
}
}

path_msgs::PathSequence Planner::simplifyPath(const path_msgs::PathSequence &path_raw)
{
    path_msgs::PathSequence result = path_raw;

    for(path_msgs::DirectionalPath& segment : result.paths) {

        for(std::size_t i = 1; i < segment.poses.size() - 1;) {
            // check if i can be removed
            if(isFree(map_info, cost_map, segment.poses[i-1].pose.position, segment.poses[i+1].pose.position)) {
                segment.poses.erase(segment.poses.begin() + i);
            } else {
                ++i;
            }
        }

    }
    return result;
}

path_msgs::PathSequence Planner::interpolatePath(const path_msgs::PathSequence &path_raw, double max_distance)
{
    path_msgs::PathSequence result;
    result.header = path_raw.header;


    for(const path_msgs::DirectionalPath& path : path_raw.paths) {
        result.paths.emplace_back();
        path_msgs::DirectionalPath& segment = result.paths.back();

        segment.header = path.header;
        segment.forward = path.forward;

        unsigned n = path.poses.size();

        segment.poses.push_back(path.poses[0]);

        for(unsigned i = 1; i < n; ++i){
            const geometry_msgs::PoseStamped* current = &path.poses[i];

            // split the segment, iff it is to large
            subdividePath(segment, segment.poses.back(), *current, max_distance);

            // add the end of the segment (is not done, when splitting)
            segment.poses.push_back(*current);
        }
    }

    return result;
}

path_msgs::PathSequence Planner::smoothPath(const path_msgs::PathSequence& path_raw, double weight_data, double weight_smooth, double tolerance) {
    path_msgs::PathSequence result;
    result.header = path_raw.header;

    for(const path_msgs::DirectionalPath& path : path_raw.paths) {
        path_msgs::DirectionalPath smoothed_segment = smoothPathSegment(path, weight_data, weight_smooth, tolerance);
        result.paths.push_back(smoothed_segment);
    }
    return result;
}

void Planner::publishGradient(const cv::Mat& gx, const cv::Mat& gy)
{
    visualization_msgs::Marker gradient_arrow;
    gradient_arrow.header.frame_id = world_frame_;
    gradient_arrow.header.stamp = ros::Time();
    gradient_arrow.ns = "planning/gradient";
    gradient_arrow.id = 0;
    gradient_arrow.type = visualization_msgs::Marker::ARROW;
    gradient_arrow.action = visualization_msgs::Marker::ADD;
    gradient_arrow.pose.orientation.w = 1.0;
    gradient_arrow.scale.x = 0.02;
    gradient_arrow.scale.y = 0.02;
    gradient_arrow.scale.z = 0.02;
    gradient_arrow.color.a = 0.75;
    gradient_arrow.color.r = 0.0;
    gradient_arrow.color.g = 0.0;
    gradient_arrow.color.b = 0.0;

    visualization_msgs::MarkerArray array;

    double res = map_info->getResolution();
    auto o = map_info->getOrigin();
    double ox = o.x;
    double oy = o.y;

    int step = 5;
    for(int y = 0; y < gx.rows; y += step) {
        for(int x = 0; x < gx.cols; x += step) {
            auto arrow = gradient_arrow;
            arrow.pose.position.x = x * res + ox;
            arrow.pose.position.y = y * res + oy;

            int grad_x = -gx.at<float>(y, x);
            int grad_y = -gy.at<float>(y, x);

            double magnitude = hypot(grad_x, grad_y) / 255.0;
            arrow.scale.x = magnitude * 10.0;

            arrow.color.r = std::min(1.0, 16.0 * magnitude);
            arrow.color.g = 1.0 - arrow.color.r;

            double angle = std::atan2(grad_y, grad_x);
            arrow.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            array.markers.push_back(arrow);

            ++gradient_arrow.id;
        }
    }

    viz_array_pub.publish(array);
}

void Planner::calculateGradient(cv::Mat &gx, cv::Mat &gy)
{
    const nav_msgs::OccupancyGrid& cost = cost_map;

    cv::Mat cost_mat(cost.info.height, cost.info.width, CV_8UC1, (uint8_t*)(cost.data.data()));

    /// Sobel Version
    //    double s = 1e-2;
    //    cv::Sobel(cost_mat, gx, CV_32F, 1, 0, 5, s);
    //    cv::Sobel(cost_mat, gy, CV_32F, 0, 1, 5, s);

    /// Minimum Search
    gx = cv::Mat(cost.info.height, cost.info.width, CV_32FC1, cv::Scalar::all(0));
    gy = cv::Mat(cost.info.height, cost.info.width, CV_32FC1, cv::Scalar::all(0));

    int d = 5;

    for(int y = 0; y < gx.rows; ++y) {
        for(int x = 0; x < gx.cols; ++x) {
            int cost = cost_mat.at<uchar>(y,x);

            if(cost == 0) {
                continue;
            }

            int min_x, min_y;
            int min_cost = 512;
            float min_dist = 2 * d;

            int starty = std::max(0, y - d);
            int endy = std::min(gx.rows - 1, y + d);
            for(int ny = starty; ny < endy; ++ny) {

                int startx = std::max(0, x - d);
                int endx = std::min(gx.rows - 1, x + d);
                for(int nx = startx; nx < endx; ++nx) {

                    int ncost = cost_mat.at<uchar>(ny,nx);
                    float dist_n = std::hypot(float(x-nx), float(y-ny));
                    if(ncost < min_cost || (ncost == min_cost && dist_n < min_dist)) {
                        min_x = nx;
                        min_y = ny;

                        min_cost = ncost;
                        min_dist = dist_n;
                    }
                }
            }

            if(min_cost < 512) {
                int dx = (x - min_x);
                int dy = (y - min_y);

                if(dx != 0 || dy != 0) {
                    float norm = std::hypot((float) dx, (float) dy);
                    float len = cost / 10.0;// std::abs(min_cost - cost) / 10.0;

                    float f = len / norm;// / norm;

                    dx *= f;
                    dy *= f;

                    gx.at<float>(y,x) = dx;
                    gy.at<float>(y,x) = dy;
                }
            }
        }
    }
}

path_msgs::PathSequence Planner::optimizePathCost(const path_msgs::PathSequence& path_raw) {
    if(!map_info) {
        return path_raw;
    }

    path_msgs::PathSequence new_path(path_raw);

    cv::Mat gx, gy;
    calculateGradient(gx, gy);

    if(publish_gradient_) {
        publishGradient(gx, gy);
    }

    double last_change = -2 * cost_optimization_tolerance;
    double change = 0;

    int offset = 2;


    for(path_msgs::DirectionalPath& segment : new_path.paths) {
        unsigned n = segment.poses.size();
        path_msgs::DirectionalPath optimized_segment = segment;

        std::vector<double> gradients_x(segment.poses.size());
        std::vector<double> gradients_y(segment.poses.size());
        std::vector<double> magnitudes(segment.poses.size());

        std::vector<double> dist_to_start(segment.poses.size());
        std::vector<double> dist_to_goal(segment.poses.size());

        std::vector<double> X(segment.poses.size());
        std::vector<double> Y(segment.poses.size());

        std::vector<double> lengths(segment.poses.size()-1);

        while(change > last_change + cost_optimization_tolerance) {
            last_change = change;
            change = 0;

            for(unsigned i = 0; i < n; ++i) {
                unsigned int x, y;
                Pose2d pt_i = convert(optimized_segment.poses[i]);
                map_info->point2cell(pt_i.x, pt_i.y, x, y);
                X[i] = x;
                Y[i] = y;
            }

            for(unsigned i = 0; i < n-1; ++i) {
                lengths[i] = std::hypot(X[i] - X[i+1], Y[i] - Y[i+1]);
            }

            dist_to_start[0] = 0.0;
            for(unsigned i = 0; i < n-1; ++i) {
                dist_to_start[i+1] = dist_to_start[i] + lengths[i];
            }

            dist_to_goal[n-1] = 0.0;
            for(unsigned i = n-1; i > 0; --i) {
                dist_to_goal[i-1] = dist_to_goal[i] + lengths[i-1];
            }

            for(unsigned i = 0; i < n; ++i){
                unsigned int x = X[i];
                unsigned int y = Y[i];
                int grad_x = gx.at<float>(y, x);
                int grad_y = gy.at<float>(y, x);
                double magnitude = hypot(grad_x, grad_y) / 255.0;
                gradients_x[i] = grad_x;
                gradients_y[i] = grad_y;
                magnitudes[i] = magnitude;
            }

            for(unsigned i = offset; i < n-offset; ++i){
                Pose2d path_i = convert(segment.poses[i]);
                Pose2d new_path_i = convert(optimized_segment.poses[i]);
                Pose2d new_path_ip1 = convert(optimized_segment.poses[i+1]);
                Pose2d new_path_im1 = convert(optimized_segment.poses[i-1]);

                double dist_border = std::min(dist_to_start[i], dist_to_goal[i]);
                double border_damp = 1.0 + 5.0 / (0.01 + (dist_border / 5.0));

                Pose2d deltaData = border_damp * cost_optimization_weight_data * (path_i - new_path_i);
                new_path_i = new_path_i + deltaData;

                double grad_x = gradients_x[i];
                double grad_y = gradients_y[i];
                Pose2d deltaCost;
                if(std::abs(grad_x) > 1e-3 && std::abs(grad_y) > 1e-3) {
                    double magnitude = 0;
                    int width = 0;
                    for(int o = -offset+1;
                        o <= offset-1; ++o) {
                        magnitude += magnitudes[i+o];
                        ++width;
                    }

                    magnitude /= width;

                    Pose2d grad(-grad_x, -grad_y, 0.0);
                    deltaCost =  cost_optimization_weight_cost * magnitude * grad;
                    new_path_i = new_path_i + deltaCost;
                }

                Pose2d deltaSmooth =  cost_optimization_weight_smooth * (new_path_ip1 + new_path_im1 - 2* new_path_i);
                new_path_i = new_path_i + deltaSmooth;

                optimized_segment.poses[i].pose.position.x = new_path_i.x;
                optimized_segment.poses[i].pose.position.y = new_path_i.y;

                change += deltaData.distance_to_origin()
                        + deltaSmooth.distance_to_origin()
                        + deltaCost.distance_to_origin();
            }
        }

        segment = optimized_segment;
    }

    return new_path;
}

Pose2d Planner::convert(const geometry_msgs::PoseStamped& rhs)
{
    return Pose2d(rhs.pose.position.x, rhs.pose.position.y, tf::getYaw(rhs.pose.orientation));
}

void Planner::laserCallback(const sensor_msgs::LaserScanConstPtr &scan, bool front)
{
    if(front) {
        scan_front = *scan;
    } else {
        scan_back = *scan;
    }
}

void Planner::integrateLaserScan(const sensor_msgs::LaserScan &scan)
{
    tf::StampedTransform trafo = lookupTransform(world_frame_, scan.header.frame_id, scan.header.stamp);

    int OBSTACLE = is_cost_map_ ? 254 : 100;

    double angle = scan.angle_min;
    for(std::size_t i = 0, total = scan.ranges.size(); i < total; ++i) {
        const float& range = scan.ranges[i];
        if(range > scan.range_min && range < (scan.range_max - 1.0) && range == range) {

            tf::Vector3 pt_laser(std::cos(angle) * range, std::sin(angle) * range, 0);
            tf::Vector3 pt_map = trafo * pt_laser;

            unsigned int x,y;
            if(map_info->point2cell(pt_map.x(), pt_map.y(), x, y)) {
                map_info->setValue(x,y, OBSTACLE);
            }
        }

        angle += scan.angle_increment;
    }
}

void Planner::growObstacles(const path_msgs::PlanPathGoal& request, double radius)
{
    lib_path::Pose2d from_world, from_map;
    transformPose(request.use_start ? request.start : lookupPose(), from_world, from_map);
    lib_path::Pose2d to_world, to_map;
    transformPose(request.goal.pose, to_world, to_map);

    cv::Mat map(map_info->getHeight(), map_info->getWidth(), CV_8UC1, map_info->getData());
    cv::Mat working;
    map.copyTo(working);

    int r = radius / map_info->getResolution();
    int iterations = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                 cv::Size( 2*r + 1, 2*r+1 ),
                                                 cv::Point( r, r ) );
    cv::dilate(working, working, element, cv::Point(-1,-1), iterations);


    cv::Mat mask(working.rows, working.cols, CV_8UC1, cv::Scalar::all(255));

    cv::circle(mask, cv::Point(from_map.x, from_map.y), r, cv::Scalar::all(0), CV_FILLED);
    cv::circle(mask, cv::Point(to_map.x, to_map.y), r, cv::Scalar::all(0), CV_FILLED);


    working.copyTo(map, mask);
}


void Planner::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    cloud_ = *cloud;
}

void Planner::integratePointCloud(const sensor_msgs::PointCloud2 &cloud)
{
    tf::StampedTransform trafo = lookupTransform(world_frame_, cloud.header.frame_id, cloud.header.stamp);

    int OBSTACLE = is_cost_map_ ? 254 : 100;

    pcl::PointCloud<pcl::PointXYZL>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::fromROSMsg(cloud, *ptr);

    for(pcl::PointCloud<pcl::PointXYZL>::const_iterator it = ptr->begin(); it != ptr->end(); ++it) {
        const pcl::PointXYZL& pt = *it;

        tf::Vector3 pt_cloud(pt.x, pt.y, pt.z);
        tf::Vector3 pt_map = trafo * pt_cloud;

        unsigned int x,y;
        if(map_info->point2cell(pt_map.x(), pt_map.y(), x, y)) {
            map_info->setValue(x,y, OBSTACLE);
        }
    }
}

void Planner::publish(const path_msgs::PathSequence &path, const path_msgs::PathSequence &path_raw)
{
    if(!path_raw.paths.empty()) {
        nav_msgs::Path path_raw_ros;

        path_raw_ros.header = path_raw.header;
        for(const path_msgs::DirectionalPath& segment : path_raw.paths) {
            path_raw_ros.poses.insert(path_raw_ros.poses.end(), segment.poses.begin(), segment.poses.end());
        }

        raw_path_publisher_.publish(path_raw_ros);
    }

    if(!path_raw.paths.empty()) {
        path_publisher_.publish(path);
        visualizePath(path_raw);
    }
}

path_msgs::DirectionalPath Planner::smoothPathSegment(const path_msgs::DirectionalPath& path, double weight_data, double weight_smooth, double tolerance)
{
    path_msgs::DirectionalPath new_path(path);
    new_path.header = path.header;
    new_path.forward = path.forward;

    int n = path.poses.size();

    double last_change = -2 * tolerance;
    double change = 0;

    int offset = 2;

    while(change > last_change + tolerance) {
        last_change = change;
        change = 0;

        for(int i = offset; i < n-offset; ++i){
            Pose2d path_i = convert(path.poses[i]);
            Pose2d new_path_i = convert(new_path.poses[i]);
            Pose2d new_path_ip1 = convert(new_path.poses[i+1]);
            Pose2d new_path_im1 = convert(new_path.poses[i-1]);

            Pose2d deltaData = weight_data * (path_i - new_path_i);
            new_path_i = new_path_i + deltaData;

            Pose2d deltaSmooth =  weight_smooth * (new_path_ip1 + new_path_im1 - 2* new_path_i);
            new_path_i = new_path_i + deltaSmooth;

            new_path.poses[i].pose.position.x = new_path_i.x;
            new_path.poses[i].pose.position.y = new_path_i.y;

            change += deltaData.distance_to_origin() + deltaSmooth.distance_to_origin();
        }
    }

    // update orientations
    double a = tf::getYaw(new_path.poses[0].pose.orientation);
    geometry_msgs::Pose current = new_path.poses[0].pose;
    geometry_msgs::Pose next = new_path.poses[1].pose;

    double dx = next.position.x - current.position.x;
    double dy = next.position.y - current.position.y;

    Eigen::Vector2d looking_dir_normalized(std::cos(a), std::sin(a));
    Eigen::Vector2d delta(dx, dy);
    const double theta_diff = std::acos(delta.dot(looking_dir_normalized) / delta.norm());

    // decide whether to drive forward or backward
    bool is_backward = (theta_diff > M_PI_2 || theta_diff < -M_PI_2) ;

    for(int i = 1; i < n-1; ++i){
        Pose2d next = convert(new_path.poses[i+1]);
        Pose2d prev = convert(new_path.poses[i-1]);

        Pose2d delta = next - prev;
        double angle = std::atan2(delta.y, delta.x);

        if(is_backward) {
            angle = MathHelper::AngleClamp(angle + M_PI);
        }

        new_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    }

    return new_path;
}

path_msgs::PathSequence Planner::empty() const
{
    path_msgs::PathSequence res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = world_frame_;
    return res;
}
