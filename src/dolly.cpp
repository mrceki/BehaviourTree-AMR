#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "print_dolly.h"
#include "dolly_client.h"
#include "stop_dolly_client.h"
#include "docking.h"
#include "move_base_client.h"

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

  auto tree = factory.createTreeFromFile("/home/cenk/Desktop/docking.xml"); // auto tree = factory.createTreeFromText(xml_text);
  StdCoutLogger logger_cout(tree);

  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();

  return 0;
}