#ifndef PRINT_DOLLY_H
#define PRINT_DOLLY_H

struct Pose3D
{
    double x, y, yaw;
};

using namespace BT;


class PrintTarget : public SyncActionNode
{
    public:
      PrintTarget(const std::string& name, const NodeConfiguration& config) :
        SyncActionNode(name, config)
      {}

      NodeStatus tick() override
      {
        auto res = getInput<Pose3D>("target");
        if (!res)
        {
          throw RuntimeError("error reading port [target]:", res.error());
        }
        Pose3D target = res.value();

        printf("Target positions: [ %.2f, %.2f, %.2f ]\n", target.x, target.y, target.yaw);
        return NodeStatus::FAILURE;
      }
      static PortsList providedPorts()
      {
        const char* description = "Simply print the target on console...";
        return {InputPort<Pose3D>("target", description)};
      }
};

#endif