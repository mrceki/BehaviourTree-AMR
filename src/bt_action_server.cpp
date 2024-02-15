#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/BatteryState.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
// #include "bt_amr/print_dolly.h"
// #include "bt_amr/dolly_client.h"
// #include "bt_amr/stop_dolly_client.h"
// #include "bt_amr/docking.h"
// #include "bt_amr/move_base_client.h"
// #include "bt_amr/battery_check.h"
// #include "bt_amr/is_battery_charged.h"
#include "bt_amr/lifter_position_controller.h"
#include "bt_amr/lifter_controller.h"
using namespace BT;

class BtAction
{
protected:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  std::string action_name_;
  std::string lifter_up, lifter_down;
  BehaviorTreeFactory factory;
  BT::Tree tree;
  ros::Subscriber bms_sub;
  bool lifter_position;
public:
  BtAction(std::string name) : service_(nh_.advertiseService(name, &BtAction::trigger, this)),
                               action_name_(name),
                               lifter_position(false)
  {
    
    // Register custom node types
    // factory.registerNodeType<FindDolly>("FindDolly");
    // factory.registerNodeType<AutoDockAction>("AutoDockAction");
    // factory.registerNodeType<StopDolly>("StopDolly");
    // factory.registerNodeType<MoveBase>("MoveBase");
    // factory.registerNodeType<BatteryCheck>("BatteryCheck");
    // factory.registerNodeType<IsBatteryCharged>("IsBatteryCharged");
    factory.registerNodeType<LifterController>("LifterController");
    factory.registerNodeType<LifterPositionController>("LifterPositionController");
  }

  bool trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    // Get the package path
    std::string package_path = ros::package::getPath("bt_amr");

    // Set the file paths for the behavior trees
    lifter_up = package_path + "/config/lifter_up.xml";
    lifter_down = package_path + "/config/lifter_down.xml";

    if (lifter_position == false)
    { 
      ROS_INFO("GOING UP");
      tree = factory.createTreeFromFile(lifter_up);
      lifter_position = true;
    }
    else
    {
      ROS_INFO("GOING DOWN");
      tree = factory.createTreeFromFile(lifter_down);
      lifter_position = false;
    }

    // Create loggers for the behavior tree
    StdCoutLogger logger_cout(tree);
    PublisherZMQ publisher_zmq(tree);

    ROS_INFO("Starting BT");
    tree.tickRoot();
    // Do some work here...

    ROS_INFO("Done with BT");
    res.success = true;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bt_lifter_server");

  BtAction bt_action("bt_lifter_server");
  ros::spin();

  return 0;
}