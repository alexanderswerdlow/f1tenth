#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_listener.h>

using namespace path_msgs;

/**
 * @brief Simple high level control dummy for testing.
 *
 * This dummy subscribes for the goals set manually in rviz. The goals are then wrapped in an action and send to
 * path_control.
 */

class HighDummy
{
public:
    HighDummy(ros::NodeHandle &nh):
        nh_(nh),
        pnh_("~"),
        client_("navigate_to_goal", true)
    {
        srand(ros::Time::now().toNSec());

        // topic for goal position
        goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("rviz_goal", 0, &HighDummy::goalCb, this);
        client_.waitForServer();

        speech_pub_ = nh.advertise<std_msgs::String>("speech", 0);

        std::string default_world_frame = nh_.param("gerona/world_frame", std::string("map"));
        pnh_.param("world_frame", world_frame_, default_world_frame);

        // target speed
        pnh_.param("target_speed", target_speed_, 1.0);

        // failure mode; possible: ABORT, REPLAN
        std::string failure_mode = "ABORT";
        pnh_.param("failure_mode", failure_mode, failure_mode);

        std::transform(failure_mode.begin(), failure_mode.end(), failure_mode.begin(), ::toupper);

        if(failure_mode == "ABORT") {
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT;
        } else if(failure_mode == "REPLAN") {
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_REPLAN;
        } else {
            ROS_WARN_STREAM("failure mode " << failure_mode << " is unknown. Defaulting to ABORT");
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT;
        }



        ROS_INFO_STREAM("listening for goal @ " << goal_sub_.getTopic());
        ROS_INFO_STREAM("failure mode is " << failure_mode);

        ROS_INFO("Client is set up");
    }


private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    actionlib::SimpleActionClient<path_msgs::NavigateToGoalAction> client_;
    ros::Subscriber goal_sub_;
    ros::Publisher speech_pub_;

    tf::TransformListener tfl_;

    std::string world_frame_;
    double target_speed_;
    int failure_mode_;

    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const path_msgs::NavigateToGoalResultConstPtr& result)
    {
        ROS_INFO("DONE with action state %s", state.toString().c_str());
        if (!result) {
            say("mission failed");
            ROS_ERROR("Lost connection to path control.");
        } else if (result->reached_goal) {
            ROS_INFO("Successfully reached goal :)");
            say("mission accomplished");
        } else {
            say("mission failed");

            ROS_WARN("Did not reach goal :(");
            const char* status_names[] = {"OTHER_ERROR", "SUCCESS", "ABORTED", "OBSTACLE", "TIMEOUT", "LOST_PATH", "NO_PATH_FOUND"};
            ROS_INFO("Result code: %d %s", result->status, status_names[result->status]);
            ROS_INFO("Additional Text: %s", state.getText().c_str());
        }
    }

    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const path_msgs::NavigateToGoalFeedbackConstPtr& feedback)
    {
        switch (feedback->status) {
        case NavigateToGoalFeedback::STATUS_MOVING:
            ROS_INFO_THROTTLE(1, "Feedback: Moving");
            ROS_DEBUG_THROTTLE(1, "There are %zu obstacles sighted on the path.", feedback->obstacles_on_path.size());
            break;

        case NavigateToGoalFeedback::STATUS_PATH_READY:
            ROS_INFO("Feedback: Path is ready.");
            break;

        case NavigateToGoalFeedback::STATUS_OBSTACLE:
            ROS_WARN_THROTTLE(1, "Feedback: Collision.");
            break;

        case NavigateToGoalFeedback::STATUS_NO_LOCAL_PLAN:
            ROS_WARN_THROTTLE(1, "Feedback: No local plan found.");
            break;

        case NavigateToGoalFeedback::STATUS_REPLAN:
            ROS_WARN("Path is replaned.");
            break;

        case NavigateToGoalFeedback::STATUS_REPLAN_FAILED:
            ROS_ERROR("Replan failed.");
            break;

        default:
            ROS_ERROR("Feedback: Unknown status code %d", feedback->status);
            break;
        }
    }

    void goalCb(const geometry_msgs::PoseStampedConstPtr &pose)
    {
        ROS_INFO("Send goal...");

        path_msgs::NavigateToGoalGoal goal;
        goal.goal.pose = *pose;
        goal.failure_mode = failure_mode_;
        goal.follower_options.velocity = target_speed_;
        goal.goal.planning_channel.data = pnh_.param("planning_channel", std::string(""));
        goal.goal.planning_algorithm.data = pnh_.param("planning_algorithm", std::string(""));

        goal.planner_options.grow_obstacles = pnh_.param("grow_obstacles", true);
        goal.planner_options.obstacle_growth_radius = pnh_.param("obstacle_radius", 1.0);

        goal.planner_options.goal_dist_threshold = pnh_.param("planner/goal_dist_threshold", 0.05);
        goal.planner_options.goal_angle_threshold_degree = pnh_.param("planner/goal_angle_threshold", 22.5);

        goal.planner_options.reversed = pnh_.param("planner/reversed", false);

        goal.planner_options.allow_forward = pnh_.param("planner/allow_forward", true);
        goal.planner_options.allow_backward = pnh_.param("planner/allow_backward", false);

        goal.planner_options.ackermann_la = pnh_.param("planner/ackermann_la", 1.2);
        goal.planner_options.ackermann_steer_steps = pnh_.param("planner/ackermann_steer_steps", 2);
        goal.planner_options.ackermann_max_steer_angle_degree = pnh_.param("planner/ackermann_max_steer_angle", 60);
        goal.planner_options.ackermann_steer_delta_degree = pnh_.param("planner/ackermann_steer_delta", 15);

        ROS_INFO_STREAM("goal: " << goal.goal);

        if(pose->header.frame_id != world_frame_) {
            if(!tfl_.waitForTransform(world_frame_, pose->header.frame_id, pose->header.stamp, ros::Duration(10.0))) {
                ROS_ERROR_STREAM("cannot drive to goal, the transformation between " << world_frame_ << " and " << pose->header.frame_id << " is not known");
            }
            tf::StampedTransform trafo;
            tfl_.lookupTransform(world_frame_, pose->header.frame_id, pose->header.stamp, trafo);

            tf::Pose old_pose;
            tf::poseMsgToTF(pose->pose, old_pose);

            tf::Pose map_pose = trafo * old_pose;
            tf::poseTFToMsg(map_pose, goal.goal.pose.pose);

            goal.goal.pose.header.frame_id = world_frame_;
        }

        client_.cancelAllGoals();
        ros::spinOnce();
        ros::Duration(0.1).sleep();

        client_.sendGoal(goal,
                         boost::bind(&HighDummy::doneCb, this, _1, _2),
                         boost::bind(&HighDummy::activeCb, this),
                         boost::bind(&HighDummy::feedbackCb, this, _1));
    }


    void say(std::string text)
    {
        std_msgs::String str;
        str.data = text;
        speech_pub_.publish(str);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "highlevel_dummy", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    HighDummy dummy(nh);

    ros::spin();
    return 0;
}

