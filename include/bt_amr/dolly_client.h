#ifndef DOLLY_CLIENT_H
#define DOLLY_CLIENT_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dolly_pose_estimation/DollyPoseAction.h>
#include <dolly_pose_estimation/DollyPoseFeedback.h>

using namespace BT;

class FindDolly : public BT::AsyncActionNode
{
public:
    FindDolly(const std::string& name, const BT::NodeConfiguration& config);

    void feedbackCallback(const dolly_pose_estimation::DollyPoseFeedbackConstPtr &feedback);

    void activeCb();

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const dolly_pose_estimation::DollyPoseResultConstPtr& result);

    BT::NodeStatus tick() override;

    static PortsList providedPorts();

private:
    actionlib::SimpleActionClient<dolly_pose_estimation::DollyPoseAction> _client;
    bool _success;
};

#endif
