#ifndef STOP_DOLLY_H
#define STOP_DOLLY_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dolly_pose_estimation/DollyPoseAction.h>

class StopDolly : public BT::AsyncActionNode
{
public:
    StopDolly(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();
    
private:
    actionlib::SimpleActionClient<dolly_pose_estimation::DollyPoseAction> _client;
};

#endif
