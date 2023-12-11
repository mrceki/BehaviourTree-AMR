#ifndef IS_BATTERY_CHARGED_H
#define IS_BATTERY_CHARGED_H

#include <ros/ros.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sensor_msgs/BatteryState.h>

class IsBatteryCharged : public BT::ConditionNode 
{
public:
	IsBatteryCharged(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;

	static BT::PortsList providedPorts();
	
private:
    sensor_msgs::BatteryState _battery_state;
	float _battery_upper_threshold;
};

#endif
