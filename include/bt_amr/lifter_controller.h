#ifndef LIFTER_CONTROLLER_H
#define LIFTER_CONTROLLER_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hamal_custom_interfaces/HomingOperationAction.h>

struct LifterGoal
{
    int target;
};

class LifterController : public BT::AsyncActionNode
{
public:
    LifterController(const std::string& name, const BT::NodeConfiguration& config);

    void feedbackCallback(const hamal_custom_interfaces::HomingOperationFeedbackConstPtr &feedback);

    void activeCb();

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const hamal_custom_interfaces::HomingOperationResultConstPtr& result);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    actionlib::SimpleActionClient<hamal_custom_interfaces::HomingOperationAction> _client;
    bool _success;
};

#endif
