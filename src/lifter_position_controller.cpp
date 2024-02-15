#include "bt_amr/lifter_position_controller.h"

LifterPositionController::LifterPositionController(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config),
      _client("lifter_action_server", true),
      _success(false)
{
}

void LifterPositionController::feedbackCallback(const hamal_custom_interfaces::LifterOperationFeedbackConstPtr &feedback)
{
    setStatus(BT::NodeStatus::RUNNING);
}

void LifterPositionController::activeCb()   
{
    ROS_INFO("Lifter Position Controller is active.");
}

void LifterPositionController::doneCb(const actionlib::SimpleClientGoalState& state,
            const hamal_custom_interfaces::LifterOperationResultConstPtr& result)
{
    ROS_INFO("Lifter Position Controller is done.");
    _success = result->target_reached;
    setStatus(BT::NodeStatus::SUCCESS);
}

BT::NodeStatus LifterPositionController::tick()
{
    BT::Optional<std::string> target_str = getInput<std::string>("position");
    if (!target_str)
    {
        ROS_ERROR("Failed to get input target value.");
        return BT::NodeStatus::FAILURE;
    }

    float target_value = std::stof(*target_str);

    if (!ros::ok())
    {
        return BT::NodeStatus::FAILURE;
    }

    ROS_WARN("Waiting for action server to start.");
    _client.waitForServer(); // Will wait for infinite time

    if (!_client.isServerConnected())
    {
        ROS_ERROR("Lifter Position Controller server is not connected.");
        return BT::NodeStatus::FAILURE;
    }

    hamal_custom_interfaces::LifterOperationGoal msg;
    msg.target_position=target_value;

    ROS_INFO("Sending lifter goal");

    _client.sendGoal(msg,
                     boost::bind(&LifterPositionController::doneCb, this, _1, _2),
                     boost::bind(&LifterPositionController::activeCb, this),
                     boost::bind(&LifterPositionController::feedbackCallback, this, _1));

    // return BT::NodeStatus::RUNNING;
}

BT::PortsList LifterPositionController::providedPorts()
{
    return{ BT::InputPort<std::string>("position") };
}
