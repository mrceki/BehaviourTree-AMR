#ifndef LIFTER_POSITION_CONTROLLER_H
#define LIFTER_POSITION_CONTROLLER_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hamal_custom_interfaces/LifterOperationAction.h>

struct LifterPositionGoal
{
    float position;
};

class LifterPositionController : public BT::AsyncActionNode
{
public:
    LifterPositionController(const std::string& name, const BT::NodeConfiguration& config);

    void feedbackCallback(const hamal_custom_interfaces::LifterOperationFeedbackConstPtr &feedback);

    void activeCb();

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const hamal_custom_interfaces::LifterOperationResultConstPtr& result);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    actionlib::SimpleActionClient<hamal_custom_interfaces::LifterOperationAction> _client;
    bool _success;
};

#endif
