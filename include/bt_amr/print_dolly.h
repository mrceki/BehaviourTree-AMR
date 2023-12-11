#ifndef PRINT_DOLLY_H
#define PRINT_DOLLY_H
#include "behaviortree_cpp_v3/bt_factory.h"
struct Pose3D
{
    double x, y, yaw;
};

class PrintTarget : public BT::SyncActionNode
{
public:
    PrintTarget(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:
    Pose3D target;
};

#endif
