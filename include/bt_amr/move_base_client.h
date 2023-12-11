#ifndef MOVEBASE_CLIENT_H
#define MOVEBASE_CLIENT_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviortree_cpp_v3/action_node.h"

// Custom type
struct Pose2D
{
    double x, y, quaternion_z, quaternion_w;
};

class MoveBase : public BT::AsyncActionNode
{
public:
    MoveBase(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    virtual void halt() override;

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient _client;
    bool _aborted;

    std::string goal_a;
    std::string goal_b;
    std::string charging_dock;
};

#endif
