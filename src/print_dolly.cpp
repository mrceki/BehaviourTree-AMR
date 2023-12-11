#include "bt_amr/print_dolly.h"

PrintTarget::PrintTarget(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PrintTarget::tick()
{
    auto res = getInput<Pose3D>("target");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    target = res.value();

    printf("Target positions: [ %.2f, %.2f, %.2f ]\n", target.x, target.y, target.yaw);
    return BT::NodeStatus::FAILURE;
}

BT::PortsList PrintTarget::providedPorts()
{
    const char *description = "Simply print the target on console...";
    return {BT::InputPort<Pose3D>("target", description)};
}
