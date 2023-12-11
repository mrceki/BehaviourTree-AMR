#include "bt_amr/dolly_client.h"

FindDolly::FindDolly(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config),
      _client("dolly_pose_estimation_server", true),
      _success(false)
{
}

void FindDolly::feedbackCallback(const dolly_pose_estimation::DollyPoseFeedbackConstPtr &feedback)
{
    _success = feedback->success;
    if (_success)
    {
        setStatus(BT::NodeStatus::SUCCESS);
    }
    else
    {
        setStatus(BT::NodeStatus::FAILURE);
    }
}

void FindDolly::activeCb()
{
    ROS_INFO("Dolly Pose Estimation is active.");
}

void FindDolly::doneCb(const actionlib::SimpleClientGoalState& state,
            const dolly_pose_estimation::DollyPoseResultConstPtr& result)
{
    ROS_INFO("Dolly Pose Estimation is done.");
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        _success = result->success;
    }
}

BT::NodeStatus FindDolly::tick()
{
    if (!ros::ok())
    {
        return BT::NodeStatus::FAILURE;
    }

    ROS_WARN("Waiting for action server to start.");
    _client.waitForServer(); // Will wait for infinite time

    if (!_client.isServerConnected())
    {
        ROS_ERROR("Dolly Pose Estimation server is not connected.");
        return BT::NodeStatus::FAILURE;
    }

    dolly_pose_estimation::DollyPoseGoal goal;
    goal.cancel = false;

    ROS_INFO("Sending empty goal to start Dolly Pose Estimation...");

    _client.sendGoal(goal,
                     boost::bind(&FindDolly::doneCb, this, _1, _2),
                     boost::bind(&FindDolly::activeCb, this),
                     boost::bind(&FindDolly::feedbackCallback, this, _1));

    // return BT::NodeStatus::RUNNING;
}

PortsList FindDolly::providedPorts()
{
    return {};
}
