#include <ros/ros.h>
#include <ros/package.h> 
#include <std_srvs/Trigger.h>
#include <sensor_msgs/BatteryState.h>
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

using namespace BT;

class BtAction
{
protected:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  std::string action_name_;
  std::string dolly_bt_tree, charging_station_tree;
  BehaviorTreeFactory factory;
  BT::Tree tree; 
  ros::Subscriber bms_sub;

public:
  BtAction(std::string name) :
    service_(nh_.advertiseService(name, &BtAction::trigger, this)),
    action_name_(name)
  {
    // Register custom node types
    factory.registerNodeType<FindDolly>("FindDolly");
    factory.registerNodeType<AutoDockAction>("AutoDockAction");
    factory.registerNodeType<StopDolly>("StopDolly");
    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<BatteryCheck>("BatteryCheck");
    factory.registerNodeType<IsBatteryCharged>("IsBatteryCharged");
  }

  bool trigger(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    // Get the package path
    std::string package_path = ros::package::getPath("bt_amr");

    // Set the file paths for the behavior trees
    charging_station_tree = package_path + "/config/charging_station_tree.xml";
    dolly_bt_tree = package_path + "/config/dolly_bt_tree.xml";
  
    ROS_INFO("Starting BT");

    // Wait for a single BatteryState message on the /battery_state topic
    sensor_msgs::BatteryState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_state", nh_);

    if (msg != nullptr)
    {
      ROS_INFO("Battery percentage: %f", msg->percentage);

      // Choose the behavior tree based on battery percentage
      if (msg->percentage < 0.1)
      {
        ROS_INFO("Battery is low, switching to tree 1");
        tree = factory.createTreeFromFile(charging_station_tree);
      }
      else
      {
        ROS_INFO("Battery is high, switching to tree 2");
        tree = factory.createTreeFromFile(dolly_bt_tree);
      }

      // Create loggers for the behavior tree
      StdCoutLogger logger_cout(tree);
      PublisherZMQ publisher_zmq(tree);

      ROS_INFO("Starting BT");
      tree.tickRoot();
    }
    else
    {
      ROS_WARN("No BatteryState message received");
    }

    // Do some work here...
    ROS_INFO("Done with BT");
    res.success = true;
    return true;
  }

  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
  {
    ROS_INFO("Battery percentage: %f", msg->percentage);

    // Choose the behavior tree based on battery percentage
    if (msg->percentage < 0.1)
    {
      ROS_INFO("Battery is low, switching to tree 1");
      tree = factory.createTreeFromFile(dolly_bt_tree);
    }
    else
    {
      ROS_INFO("Battery is high, switching to tree 2");
      tree = factory.createTreeFromFile(charging_station_tree);
    }

    // Create loggers for the behavior tree
    StdCoutLogger logger_cout(tree);
    PublisherZMQ publisher_zmq(tree);

    ROS_INFO("Starting BT");
    tree.tickRoot();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_docking_server");
  
  BtAction bt_action("fake_docking_server");
  ros::spin();

  return 0;
}