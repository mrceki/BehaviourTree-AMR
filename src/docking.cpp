#include "bt_amr/docking.h"

AutoDockAction::AutoDockAction(const std::string& name, const NodeConfiguration& config)
    : AsyncActionNode(name, config),
    _client("autodock_action", true)
{
}

PortsList AutoDockAction::providedPorts()
{
    return {};
}

NodeStatus AutoDockAction::tick()
{
    if (!ros::ok())
    {
        return BT::NodeStatus::FAILURE;
    }

    ROS_WARN("Waiting for docking action server to start.");
    _client.waitForServer(); // Will wait for infinite time

    if (!_client.isServerConnected())
    {
        ROS_ERROR("Docking server is not connected.");
        return BT::NodeStatus::FAILURE;
    }

    autodock_core::AutoDockingGoal goal;
    _client.sendGoal(goal);
    NodeStatus::RUNNING;

    if (_client.waitForResult(ros::Duration(80.0))) {
        if (_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return NodeStatus::SUCCESS;
        } else {
            return NodeStatus::FAILURE;
        }
    } else {
        return NodeStatus::FAILURE;
    }
}
