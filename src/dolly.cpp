#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "print_dolly.h"
#include "dolly_client.h"


using namespace BT;

// static const char* xml_text = R"(

//  <root BTCPP_format="4" >
//      <BehaviorTree ID="MainTree">
//         <KeepRunningUntilFailure>
//            <Sequence name="root">
//                <FindDolly       goal="{GoalPosition}" />
//                <PrintTarget     target="{GoalPosition}" />
//            </Sequence>
//         </KeepRunningUntilFailure>
//      </BehaviorTree>
//  </root>
//  )";
BehaviorTreeFactory factory;



// clang-format on

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dolly_bt");
  using namespace BT;

  BehaviorTreeFactory factory;
  factory.registerNodeType<FindDolly>("FindDolly");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromFile("/home/cenk/Desktop/dolly.xml"); // auto tree = factory.createTreeFromText(xml_text);
  StdCoutLogger logger_cout(tree);

  // This logger publish status changes using ZeroMQ. Used by Groot
  PublisherZMQ publisher_zmq(tree);
  tree.tickRootWhileRunning();

  return 0;
}