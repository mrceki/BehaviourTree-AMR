#include "bt_amr/move_base_client.h"

MoveBase::MoveBase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config),
      _client("move_base", true)
{
}


BT::NodeStatus MoveBase::tick()
{
    // If no server is present, fail after 2 seconds
    if (!_client.waitForServer(ros::Duration(2.0))) {
        ROS_ERROR("Can't contact move_base server");
        return BT::NodeStatus::FAILURE;
    }

    auto text = getInput<std::string>("goal");
    std::cout << "Sending the robot to: " << text.value() << std::endl;

    Pose2D goal;
    if (text.value() == "dolly_0") {
        // Set goal_a values
    } else if (text.value() == "dolly_1") {
        // Set goal_b values
    } else if (text.value() == "charging_dock") {
        // Set charging_dock values
    } else {
        ROS_INFO("Wrong goal name in the BT xml file!!!!");
        return BT::NodeStatus::FAILURE;
    }

    setOutput<Pose2D>("goal", goal);

    // Reset this flag
    _aborted = false;

    // Build the message from Pose2D
    move_base_msgs::MoveBaseGoal msg;
    msg.target_pose.header.frame_id = "map";
    msg.target_pose.header.stamp = ros::Time::now();
    msg.target_pose.pose.position.x = goal.x;
    msg.target_pose.pose.position.y = goal.y;
    msg.target_pose.pose.orientation.z = goal.quaternion_z;
    msg.target_pose.pose.orientation.w = goal.quaternion_w;

    _client.sendGoal(msg);

    while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
        // Polling at 50 Hz. No big deal in terms of CPU
    }

    if (_aborted) {
        ROS_ERROR("MoveBase aborted");
        return BT::NodeStatus::FAILURE;
    }

    if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("MoveBase failed");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("Target reached");
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveBase::providedPorts()
{
    return{ BT::InputPort<std::string>("goal") };
}

void MoveBase::halt()
{
    _aborted = true;
}
