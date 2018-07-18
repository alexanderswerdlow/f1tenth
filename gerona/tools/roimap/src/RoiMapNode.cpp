/*
 *  GradientCostmapNode.cpp
 *
 *  Created on: Mar 11, 2014
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>

class ROIMapNode
{
public:

    ROIMapNode(ros::NodeHandle &nh)
        : running_avg_(0), running_avg_ticks_(0)
    {
        std::string map_topic ("map/hector");
        std::string map_service ("/dynamic_map/hector");
        std::string map_topic_result ("map");
        std::string map_service_result ("/dynamic_map");
        nh.param("topic_map", map_topic, map_topic);
        nh.param("map_service", map_service, map_service);
        nh.param("topic_map_result", map_topic_result, map_topic_result);
        nh.param("map_service_result", map_service_result, map_service_result);

        nh.param("padding", padding_, 0.0);
        nh.param("null", null_, -1);

        map_subscriber_ = nh.subscribe<nav_msgs::OccupancyGrid> (map_topic, 10, boost::bind(&ROIMapNode::updateMapCallback, this, _1));
        map_publisher_  = nh.advertise<nav_msgs::OccupancyGrid> (map_topic_result, 10, true);


        map_service_client = nh.serviceClient<nav_msgs::GetMap> (map_service);
        map_service_ = nh.advertiseService (map_service_result, &ROIMapNode::getMap, this);
    }

    bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
    {
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            if(!updateMap(map_service.response.map)) {
                return false;
            }

            res.map = current_map_;
            return true;
        } else {
            return false;
        }
    }

    void updateMapCallback(const nav_msgs::OccupancyGridConstPtr &ptr)
    {
        try {
            if(!updateMap(*ptr)) {
                return;
            }

            map_publisher_.publish(current_map_);
        } catch(const std::exception& e) {
            ROS_ERROR_STREAM("shrinking map failed with " << e.what());
        }
    }

    bool updateMap(const nav_msgs::OccupancyGrid &map)
    {
        map_data_ = map.data;

        ros::Time start = ros::Time::now();

        cv::Mat working(map.info.height, map.info.width, CV_8SC1, map_data_.data());
        cv::Point min(map.info.width, map.info.height);
        cv::Point max(-1,-1);

        bool empty = true;
        int8_t* ptr = working.ptr<int8_t>();
        for(int y = 0 ; y < working.rows ; ++y) {
            for(int x = 0 ; x < working.cols ; ++x) {
                if(ptr[y * working.cols + x] != null_) {
                   min.x = std::min(min.x, x);
                   min.y = std::min(min.y, y);
                   max.x = std::max(max.x, x);
                   max.y = std::max(max.y, y);
                   empty = false;
                }
            }
        }

        if(empty) {
            return false;
        }


        int padding = std::floor(padding_ / map.info.resolution + 0.5);
        min.x = std::max(min.x - padding, 0);
        min.y = std::max(min.y - padding, 0);
        max.x = std::min(max.x + padding, working.cols - 1);
        max.y = std::min(max.y + padding, working.rows - 1);

        int width  = max.x - min.x;
        int height = max.y - min.y;
        cv::Rect roi(min.x , min.y, width, height);

        current_map_.data.resize(width * height);
        int8_t *data_ptr = current_map_.data.data();
        for(int y = 0 ; y < height ; ++y) {
            for(int x = 0 ; x < width ; ++x) {
                 data_ptr[y * width + x] = ptr[(min.y + y) * working.cols + (min.x + x)];
            }
        }

        current_map_.info                    = map.info;
        current_map_.info.height             = height;
        current_map_.info.width              = width;
        current_map_.info.origin.position.x += roi.x * map.info.resolution;
        current_map_.info.origin.position.y += roi.y * map.info.resolution;

        ros::Duration diff = ros::Time::now() - start;
        double diff_ms =  diff.toNSec() * 1e-6;
        running_avg_ticks_++;
        running_avg_ = (running_avg_ * (running_avg_ticks_-1) / running_avg_ticks_) + diff_ms / running_avg_ticks_;
        ROS_INFO_STREAM("map shrink took " << diff_ms << "ms , sampling: " << sampling_ << "] [avg. " << running_avg_ << "ms]");

        return true;
    }


private:
    ros::Subscriber     map_subscriber_;
    ros::Publisher      map_publisher_;

    ros::ServiceClient  map_service_client;
    ros::ServiceServer  map_service_;

    nav_msgs::OccupancyGrid current_map_;
    std::vector<int8_t>     map_data_;

    int     null_;

    double  running_avg_;
    int     running_avg_ticks_;
    int     sampling_;

    double padding_;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "roimap");
    ros::NodeHandle n("~");

    ROIMapNode roimap(n);

    ros::spin();

    return 0;
}
