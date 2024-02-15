#include "bt_amr/lifter_controller.h"


LifterController::LifterController(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config),
      _client("lifter_homing_server", true),
      _success(false)
{
}

void LifterController::feedbackCallback(const hamal_custom_interfaces::HomingOperationFeedbackConstPtr &feedback)
{
    setStatus(BT::NodeStatus::RUNNING);
}

void LifterController::activeCb()
{
    ROS_INFO("Dolly Pose Estimation is active.");
}

void LifterController::doneCb(const actionlib::SimpleClientGoalState& state,
            const hamal_custom_interfaces::HomingOperationResultConstPtr& result)
{
    ROS_INFO("Dolly Pose Estimation is done.");
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        _success = result->homingDone;
        setStatus(BT::NodeStatus::SUCCESS);
    }
}

BT::NodeStatus LifterController::tick()
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

    hamal_custom_interfaces::HomingOperationGoal msg;
    msg.homingInfo.homingMethod = 17;

    ROS_INFO("Sending lifter goal");

    _client.sendGoal(msg,
                     boost::bind(&LifterController::doneCb, this, _1, _2),
                     boost::bind(&LifterController::activeCb, this),
                     boost::bind(&LifterController::feedbackCallback, this, _1));

    // return BT::NodeStatus::RUNNING;
}

BT::PortsList LifterController::providedPorts()
{
    return{ BT::InputPort<std::string>("target") };
}
