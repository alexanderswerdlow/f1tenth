#include "pathcontroller.h"
#include <std_msgs/String.h>

using namespace path_msgs;

PathController::PathController(ros::NodeHandle &nh):
    node_handle_(nh), private_node_handle_("~"),
    navigate_to_goal_server_(nh, "navigate_to_goal", boost::bind(&PathController::navToGoalActionCallback, this, _1), false),
    follow_path_client_("follow_path")
{
    ros::param::param<int>("~num_replan_attempts", opt_.num_replan_attempts, 5);

    speech_pub_ = node_handle_.advertise<std_msgs::String>("speech", 5);
    goal_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("move_base_simple/accepted_goal", 5);

    navigate_to_goal_server_.start();
    ROS_INFO("Initialisation done.");
}

void PathController::navToGoalActionCallback(const path_msgs::NavigateToGoalGoalConstPtr &goal)
{    
    current_goal_ = goal;

    ROS_INFO_STREAM("Start Action! Requested velocity: " << current_goal_->follower_options.velocity);

    if(current_goal_->follower_options.init_mode != FollowerOptions::INIT_MODE_CONTINUE) {
        follow_path_client_.cancelAllGoals();
    };

    ros::spinOnce(); ros::Duration(0.1).sleep();
    switch (current_goal_->failure_mode) {
    case NavigateToGoalGoal::FAILURE_MODE_ABORT:
        /// Abort mode. Simply process ones and abort, if some problem occurs.
        if (processGoal()) { // if processGoal returns false, this means, there is no result, which can be handled.
            handleFollowPathResult();
        }
        break;

    case NavigateToGoalGoal::FAILURE_MODE_REPLAN:
        /// Replan mode. If some problem occurs during path following, make new plan with same goal.
    { // need braces here, due to declaration of variables in case-block

        // stop after n replanings to avoid getting stuck
        int replan_counter = 0;
        bool failed = true;

        while(replan_counter <= opt_.num_replan_attempts) {
            ros::spinOnce();
            if (!processGoal()) {
                // Follower aborted or goal got preempted. We are finished here, result is already send.
                return;
            }

            // if follower reports success, we are done. If not, replan
            if (follow_path_result_ && follow_path_result_->status == FollowPathResult::RESULT_STATUS_SUCCESS) {
                failed = false;
                break;
            } else {
                ++replan_counter;

                ROS_WARN("Path execution failed. Replan [%d].", replan_counter);
                // send feedback
                NavigateToGoalFeedback feedback;
                feedback.status = NavigateToGoalFeedback::STATUS_REPLAN;
                navigate_to_goal_server_.publishFeedback(feedback);

                if(replan_counter <= opt_.num_replan_attempts) {
                    say("try again!");
                }
            }
        }

        if (failed) {
            ROS_WARN("Path execution failed. Max number of replan attempts reached.");
            // send feedback
            NavigateToGoalFeedback feedback;
            feedback.status = NavigateToGoalFeedback::STATUS_REPLAN_FAILED;
            navigate_to_goal_server_.publishFeedback(feedback);
        }

        handleFollowPathResult();
    }
        break;

    default:
        ROS_ERROR("Invalid failure mode %d.", goal->failure_mode);
        NavigateToGoalResult result;
        result.status = NavigateToGoalResult::STATUS_OTHER_ERROR;
        navigate_to_goal_server_.setAborted(result, "Invalid failure mode.");
        break;
    }
}

void PathController::publishGoalMessage()
{
    geometry_msgs::PoseStamped goal_msg = current_goal_->goal.pose;
    if(goal_msg.header.frame_id.empty()) {
        ROS_ERROR("invalid goal without frame id received.");
    } else {
        goal_msg.header.stamp = ros::Time::now();
        goal_pub_.publish(goal_msg);
    }
}

bool PathController::processGoal()
{
    follow_path_done_ = false;

    ROS_INFO("Wait for follow_path action server...");
    follow_path_client_.waitForServer();

    if(current_goal_->follower_options.init_mode != FollowerOptions::INIT_MODE_CONTINUE) {
        follow_path_client_.cancelAllGoals();
    }

    publishGoalMessage();

    // send goal pose to planner and wait for the result
    findPath();

    ros::spinOnce();

    // check, if path has been found
    if ( requested_path_->paths.size() == 1 && requested_path_->paths.front().poses.size() == 1) {
        ROS_WARN("path has only one pose, assuming that start and goal are equal");
        path_msgs::FollowPathResultPtr follow_path_result(new path_msgs::FollowPathResult);
        follow_path_result->status = FollowPathResult::RESULT_STATUS_SUCCESS;
        follow_path_result_ = follow_path_result;
        return true;
    }
    if ( requested_path_->paths.empty() ) {
        ROS_WARN("No path found. Abort goal.");

        NavigateToGoalResult result;
        result.reached_goal = false;
        result.status = NavigateToGoalResult::STATUS_NO_PATH_FOUND;

        navigate_to_goal_server_.setAborted(result);

        return false;
    }

    // before we're continuing, check if the goal already has been preemted to avoid unnecessary start of follow_path
    // action
    if (navigate_to_goal_server_.isPreemptRequested()) {
        ROS_INFO("Preempt goal.\n---------------------");
        navigate_to_goal_server_.setPreempted();
        return false;
    }

    // feedback about path
    {
        NavigateToGoalFeedback feedback;
        feedback.status = NavigateToGoalFeedback::STATUS_PATH_READY;
        navigate_to_goal_server_.publishFeedback(feedback);
    }

    path_msgs::FollowPathGoal path_action_goal;
    path_action_goal.path = *requested_path_;
    path_action_goal.follower_options.velocity = current_goal_->follower_options.velocity;
    path_action_goal.follower_options.init_mode = current_goal_->follower_options.init_mode;

    follow_path_client_.sendGoal(path_action_goal,
                                 boost::bind(&PathController::followPathDoneCB, this, _1, _2),
                                 boost::bind(&PathController::followPathActiveCB, this),
                                 boost::bind(&PathController::followPathFeedbackCB, this, _1));

    ros::Rate rate(60);
    while ( ! follow_path_client_.getState().isDone() ) {
        rate.sleep();
        ros::spinOnce();
        if (navigate_to_goal_server_.isPreemptRequested()) {
            ROS_INFO("Preempt goal.\n---------------------");
            if(current_goal_->follower_options.init_mode != FollowerOptions::INIT_MODE_CONTINUE) {
                follow_path_client_.cancelAllGoals();
            }
            // wait until the goal is really canceled (= done callback is called).
            //if (!waitForFollowPathDone(ros::Duration(10))) {
            //    ROS_WARN("follow_path_client does not react to cancelGoal() for 10 seconds.");
            //}
            //FIXME ^ why is this commented?

            navigate_to_goal_server_.setPreempted();

            // don't check for new goal here. If there is one, it will cause a new execution of this callback, after
            // this instance has stopped.
            return false;
        }

        // As long as only one action client is active, a new goal should automatically preempt the former goal.
        // Separately checking for new goals should only be necessary, if there are more than one client (or a client
        // that gets restarted), which is currently not intended.
        //        if (navigate_to_goal_server_.isNewGoalAvailable()) {
        //            ROS_INFO("New goal available.\n---------------------");
        //            follow_path_client_.cancelGoal();
        //            navigate_to_goal_server_.setPreempted();
        //            break;
        //        }
    }


    // wait until the action is really finished
    if (!waitForFollowPathDone(ros::Duration(10))) {
        ROS_WARN("Wait for follow_path action to be finished, but timeout expired!");
        NavigateToGoalResult result;
        result.status = NavigateToGoalResult::STATUS_TIMEOUT;
        navigate_to_goal_server_.setAborted(result, "Wait for follow_path action to be finished, but timeout expired.");
        return false;
    }
    return true;
}

void PathController::handleFollowPathResult()
{
    /*** IMPORTANT: No matter, what the result is, the navigate_to_goal action has to be finished in some way! ***/

    /// Construct result message
    path_msgs::NavigateToGoalResult nav_result;

    if (!follow_path_result_) {
        nav_result.status = NavigateToGoalResult::STATUS_OTHER_ERROR;
        nav_result.reached_goal = false;
    } else {
        nav_result.reached_goal = (follow_path_result_->status == FollowPathResult::RESULT_STATUS_SUCCESS);

        ROS_DEBUG("FollowPathResult status = %d", follow_path_result_->status);

        if (nav_result.reached_goal) {
            nav_result.status = NavigateToGoalResult::STATUS_SUCCESS;
        }
        else {
            switch (follow_path_result_->status) {
            case FollowPathResult::RESULT_STATUS_OBSTACLE:
                nav_result.status = NavigateToGoalResult::STATUS_OBSTACLE;
                break;

            case FollowPathResult::RESULT_STATUS_ABORTED:
                nav_result.status = NavigateToGoalResult::STATUS_ABORTED;
                break;

            case FollowPathResult::RESULT_STATUS_PATH_LOST:
                nav_result.status = NavigateToGoalResult::STATUS_LOST_PATH;
                break;

            case FollowPathResult::RESULT_STATUS_TIMEOUT:
                nav_result.status = NavigateToGoalResult::STATUS_TIMEOUT;
                break;

            default:
                nav_result.status = NavigateToGoalResult::STATUS_OTHER_ERROR;
                break;
            }
        }
    }


    /* Terminate navigate_to_goal action according to the final state of the the follow_path action.
     *
     * According to [1] only REJECTED, RECALLED, PREEMPTED, ABORTED and SUCCEEDED are terminal states.
     * Thus theses states should be the only ones, that can occur here.
     *
     * [1] http://wiki.ros.org/actionlib/DetailedDescription
     */
    switch (follow_path_final_state_) {
    case GoalState::REJECTED:
    case GoalState::RECALLED:
    case GoalState::ABORTED:
    case GoalState::LOST:
        navigate_to_goal_server_.setAborted(nav_result);
        break;

    case GoalState::PREEMPTED:
        // This should never happen, because this method should not be called when the goal was preemted (this is
        // handled separately in the execute-callback).
        ROS_ERROR("This function should never receive a preemted goal. This is likely a bug! [file %s, line %d]",
                  __FILE__, __LINE__);
        navigate_to_goal_server_.setAborted(nav_result);
        break;

    case GoalState::SUCCEEDED:
        navigate_to_goal_server_.setSucceeded(nav_result);
        break;

    default: // Are there other states, that should be handled somehow?
        ROS_ERROR("Unexpected final state of follow_path goal. navigate_to_goal is aborted. Maybe this is a bug. [file %s, line %d]",
                  __FILE__, __LINE__);
        navigate_to_goal_server_.setAborted(nav_result);

    }
}

void PathController::followPathDoneCB(const actionlib::SimpleClientGoalState &state,
                                      const path_msgs::FollowPathResultConstPtr &result)
{
    if (state == state.LOST) {
        ROS_ERROR("Lost connection to path follower.");
    }

    ROS_INFO("Path execution finished.\n---------------------");

    follow_path_final_state_ = state.state_;
    follow_path_result_ = result;
    follow_path_done_ = true;
}

void PathController::followPathActiveCB()
{
    ROS_INFO("Path is now active.");
    // is there anything to do here?
}

void PathController::followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback)
{
    //ROS_INFO_THROTTLE(1,"Driven distance: %g;  Distance to goal: %g", feedback->dist_driven, feedback->dist_goal);

    path_msgs::NavigateToGoalFeedback nav_feedback;

    switch(feedback->status) {
    case FollowPathFeedback::MOTION_STATUS_MOVING:
        nav_feedback.status = NavigateToGoalFeedback::STATUS_MOVING;
        break;

    case FollowPathFeedback::MOTION_STATUS_OBSTACLE:
        nav_feedback.status = NavigateToGoalFeedback::STATUS_OBSTACLE;
        break;

    case FollowPathFeedback::MOTION_STATUS_NO_LOCAL_PATH:
        nav_feedback.status = NavigateToGoalFeedback::STATUS_NO_LOCAL_PLAN;
        break;

    default:
        ROS_ERROR("Feedback: Unknown status code %d", feedback->status);
        break;
    }

    nav_feedback.obstacles_on_path = feedback->obstacles_on_path;

    navigate_to_goal_server_.publishFeedback(nav_feedback);

    publishGoalMessage();
}

void PathController::findPath()
{
    {
        // pause all activity that can be paused:
        std_msgs::String pause;
        pause.data = "pause";
        //sys_pub_.publish(pause);
    }

    const path_msgs::Goal& goal = current_goal_->goal;

    PlanPathGoal goal_msg;
    goal_msg.use_start = false;
    goal_msg.goal = goal;
    goal_msg.options = current_goal_->planner_options;

    ros::Duration timeout(20.0);
    ros::Time start = ros::Time::now();

    std::string planner_topic = goal.planning_channel.data;
    ROS_INFO_STREAM("got request with channel " << planner_topic);
    if(planner_topic.empty()) {
        planner_topic = node_handle_.resolveName("plan_path", true);
        ROS_INFO_STREAM("remapped channel to " << planner_topic);
    }

    // is this a new planner?
    auto it = path_planner_client_.find(planner_topic);
    if(it == path_planner_client_.end()) {
        path_planner_client_[planner_topic] = boost::make_shared<PlanPathClient>(planner_topic);
    }

    PlanPathClient& planner = *path_planner_client_.at(planner_topic);
    ROS_INFO_STREAM("waiting for planner @ " << planner_topic);
    planner.waitForServer();
    planner.cancelAllGoals();
    ros::spinOnce(); ros::Duration(0.1).sleep();

    planner.sendGoal(goal_msg);

    ROS_INFO("waiting for path");
    while(ros::Time::now() < start + timeout) {
        if(planner.waitForResult(ros::Duration(0.5))) {
            break;
        }

        if(navigate_to_goal_server_.isPreemptRequested()) {
            ROS_WARN("preempting");
            planner.cancelAllGoals();
            navigate_to_goal_server_.setPreempted();
            return;
        }

        ROS_INFO_THROTTLE(2, "still waiting for path");
        ros::spinOnce();
    }

    actionlib::SimpleClientGoalState state = planner.getState();
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {


        ROS_INFO("Got a path, continue");
        path_msgs::PathSequencePtr path(new path_msgs::PathSequence(planner.getResult()->path));

        requested_path_ = path;

    } else {
        say("no path found!");

        ROS_ERROR_STREAM("Path planner failed. Final state: " << state.toString());
        planner.cancelAllGoals();
        ros::spinOnce(); ros::Duration(0.1).sleep();
        requested_path_ = path_msgs::PathSequencePtr(new path_msgs::PathSequence);
    }


    {
        // unpause all activity that can be paused:
        std_msgs::String unpause;
        unpause.data = "unpause";
        //sys_pub_.publish(unpause);
    }
}

bool PathController::waitForFollowPathDone(ros::Duration timeout)
{
    ros::Time expire_time = ros::Time::now() + timeout;

    while (!follow_path_done_ && expire_time > ros::Time::now());

    return follow_path_done_;
}

void PathController::say(std::string text)
{
    std_msgs::String str;
    str.data = text;
    speech_pub_.publish(str);
}
