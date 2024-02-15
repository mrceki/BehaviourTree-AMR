#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "bt_amr/print_dolly.h"
#include "bt_amr/dolly_client.h"
#include "bt_amr/stop_dolly_client.h"
#include "bt_amr/docking.h"
#include "bt_amr/move_base_client.h"
#include "bt_amr/battery_check.h"
#include "bt_amr/is_battery_charged.h"
#include "bt_amr/lifter_controller.h"
#include "bt_amr/lifter_position_controller.h"

using namespace BT;


BehaviorTreeFactory factory;



// clang-format on

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dolly_bt");
  using namespace BT;

  BehaviorTreeFactory factory;
  factory.registerNodeType<FindDolly>("FindDolly");
  factory.registerNodeType<AutoDockAction>("AutoDockAction");
  factory.registerNodeType<StopDolly>("StopDolly");
  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<BatteryCheck>("BatteryCheck");
  factory.registerNodeType<IsBatteryCharged>("IsBatteryCharged");
  factory.registerNodeType<LifterController>("LifterController");
  factory.registerNodeType<LifterPositionController>("LifterPositionController");
  auto tree = factory.createTreeFromFile("/home/cenk/Desktop/lifter.xml"); // auto tree = factory.createTreeFromText(xml_text);
  StdCoutLogger logger_cout(tree);

  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();

  return 0;
}