#ifndef STOP_DOLLY_H
#define STOP_DOLLY_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dolly_pose_estimation/DollyPoseAction.h>

class StopDolly : public BT::AsyncActionNode
{
public:
    StopDolly(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("dolly_pose_estimation", true)
    {

    }

    BT::NodeStatus tick() override
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
            return BT::NodeStatus::SUCCESS;
        }
    }

    static PortsList providedPorts()
    {
        return {};
    }

private:
    actionlib::SimpleActionClient<dolly_pose_estimation::DollyPoseAction> _client;
};

#endif
