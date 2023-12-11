#ifndef DOCKING_ACTION_CLIENT_H
#define DOCKING_ACTION_CLIENT_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <actionlib/client/simple_action_client.h>
#include <autodock_core/AutoDockingAction.h>

using namespace BT;

class AutoDockAction : public AsyncActionNode
{
public:
    AutoDockAction(const std::string& name, const NodeConfiguration& config);
    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    actionlib::SimpleActionClient<autodock_core::AutoDockingAction> _client;
};
#endif 