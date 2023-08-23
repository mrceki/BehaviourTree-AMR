#ifndef BATTERY_CHECK_H
#define BATTERY_CHECK_H

#include <ros/ros.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sensor_msgs/BatteryState.h>

using namespace BT;

class BatteryCheck : public BT::ConditionNode 
{
public:
	BatteryCheck(const std::string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_battery_state = sensor_msgs::BatteryState();
		_battery_state.percentage = 1.0;
        _battery_lower_threshold = 0.2;
    }

    BT::NodeStatus tick() override
	{
        _battery_state = *(ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_status", ros::Duration(10.0)));
		ROS_INFO ("Battery percentage : %f ",_battery_state.percentage);
		return (_battery_state.percentage < _battery_lower_threshold) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
    sensor_msgs::BatteryState _battery_state;
	float _battery_lower_threshold;
};

#endif