#ifndef DOLLY_CLIENT_H
#define DOLLY_CLIENT_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dolly_pose_estimation/DollyPoseAction.h>
#include <dolly_pose_estimation/DollyPoseFeedback.h>

class FindDolly : public BT::AsyncActionNode
{
public:
    FindDolly(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config),
          _client("dolly_pose_estimation", true),
          _success(false)
    {

    }

    void feedbackCallback(const dolly_pose_estimation::DollyPoseFeedbackConstPtr &feedback)
    {
        _success = feedback->success;
        if (_success)
        {
            setStatus(BT::NodeStatus::SUCCESS);
        }
        else
        {
            setStatus(BT::NodeStatus::FAILURE);
        }
    }

    void activeCb()
    {
        ROS_INFO("Dolly Pose Estimation is active.");
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const dolly_pose_estimation::DollyPoseResultConstPtr& result)
    {
        ROS_INFO("Dolly Pose Estimation is done.");
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            _success = result->success;
        }
    }

    BT::NodeStatus tick() override
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

        dolly_pose_estimation::DollyPoseGoal goal;
        goal.cancel = false;

        ROS_INFO("Sending empty goal to start Dolly Pose Estimation...");

        _client.sendGoal(goal,
                         boost::bind(&FindDolly::doneCb, this, _1, _2),
                         boost::bind(&FindDolly::activeCb, this),
                         boost::bind(&FindDolly::feedbackCallback, this, _1));

    }

    static PortsList providedPorts()
    {
        return {};
    }

private:
    actionlib::SimpleActionClient<dolly_pose_estimation::DollyPoseAction> _client;
    bool _success;
};

#endif
