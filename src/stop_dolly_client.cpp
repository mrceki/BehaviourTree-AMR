#include "bt_amr/stop_dolly_client.h"

StopDolly::StopDolly(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config),
      _client("dolly_pose_estimation", true)
{
}

BT::NodeStatus StopDolly::tick()
{
    ROS_INFO("Sending cancel command to Dolly Pose Estimation...");

    dolly_pose_estimation::DollyPoseGoal goal;
    goal.cancel = true;

    _client.sendGoal(goal);

    // Wait for the cancel goal to complete
    _client.waitForResult();

    if (_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList StopDolly::providedPorts()
{
    return {};
}
