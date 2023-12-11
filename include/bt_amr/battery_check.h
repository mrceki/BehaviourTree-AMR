#ifndef BATTERY_CHECK_H
#define BATTERY_CHECK_H

#include <ros/ros.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sensor_msgs/BatteryState.h>

class BatteryCheck : public BT::ConditionNode 
{
public:
	BatteryCheck(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

	static BT::PortsList providedPorts();
	
private:
    sensor_msgs::BatteryState _battery_state;
	float _battery_lower_threshold;
};

#endif
