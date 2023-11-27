#ifndef MOVEBASE_CLIENT_H
#define MOVEBASE_CLIENT_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
// #include <tf/transform_datatypes.h>
// #include <fstream>
// #include <iostream>

// Custom type
struct Pose2D
{
    double x, y, quaternion_z, quaternion_w;
};


namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 4)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.quaternion_z = convertFromString<double>(parts[2]);
        output.quaternion_w = convertFromString<double>(parts[3]);
        return output;
    }
}
} // end namespace BT

//----------------------------------------------------------------

class MoveBase : public BT::AsyncActionNode
{
public:
    MoveBase(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("move_base", true)
    {

    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("goal") };
    }

BT::NodeStatus tick() override {
    // if no server is present, fail after 2 seconds
    if (!_client.waitForServer(ros::Duration(2.0))) {
      ROS_ERROR("Can't contact move_base server");
      return BT::NodeStatus::FAILURE;
    }

    auto text= getInput<std::string>("goal");
    std::cout<< "----------------------------------------------------------------Sending the robot to : " << text.value() << std::endl;


    Pose2D goal;
    if (text.value() == "dolly_0"){

      std::string goal_a = "4.4;3.84;0.25;1.0";
      auto parts = BT::splitString(goal_a, ';');
      goal.x     = BT::convertFromString<double>(parts[0]);
      goal.y     = BT::convertFromString<double>(parts[1]);
      goal.quaternion_z = BT::convertFromString<double>(parts[2]);
      goal.quaternion_w = BT::convertFromString<double>(parts[3]);          
    }
    else if (text.value() == "dolly_1"){

      std::string goal_b = "10.0;2.0;0.0;1.0";
      auto parts = BT::splitString(goal_b, ';');
      goal.x     = BT::convertFromString<double>(parts[0]);
      goal.y     = BT::convertFromString<double>(parts[1]);
      goal.quaternion_z = BT::convertFromString<double>(parts[2]);
      goal.quaternion_w = BT::convertFromString<double>(parts[3]);
    }
    else if (text.value() == "charging_dock"){

      std::string charging_dock = "0.386;4.669;0.0;1.0";
      auto parts = BT::splitString(charging_dock, ';');
      goal.x     = BT::convertFromString<double>(parts[0]);
      goal.y     = BT::convertFromString<double>(parts[1]);
      goal.quaternion_z = BT::convertFromString<double>(parts[2]);
      goal.quaternion_w = BT::convertFromString<double>(parts[3]);
    }
    else {
      ROS_INFO("Wrong goal name in the BT xml file!!!!");
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
      // polling at 50 Hz. No big deal in terms of CPU
    }

    if (_aborted) {
      // this happens only if method halt() was invoked
      //_client.cancelAllGoals();
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

    virtual void halt() override
    {
        _aborted = true;
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _client;
    bool _aborted;

    std::string goal_a;
    std::string goal_b;
    std::string charging_dock;
    
};
#endif