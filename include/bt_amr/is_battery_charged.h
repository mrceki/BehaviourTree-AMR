#ifndef IS_BATTERY_CHARGED
#define IS_BATTERY_CHARGED

#include <ros/ros.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sensor_msgs/BatteryState.h>

using namespace BT;

class IsBatteryCharged : public ConditionNode 
{
public:
	IsBatteryCharged(const std::string& name, const NodeConfiguration& config) :
	ConditionNode(name, config)
	{
		_battery_state = sensor_msgs::BatteryState();
		_battery_state.percentage = 1.0;
        _battery_upper_threshold = 0.8;
    }

    NodeStatus tick() override
	{
        _battery_state = *(ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_status", ros::Duration(10.0)));
		ROS_INFO ("Battery percentage : %f ",_battery_state.percentage);
		return (_battery_state.percentage > _battery_upper_threshold) ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
	}

	static PortsList providedPorts()
	{
		return {};
	}
	
private:
    sensor_msgs::BatteryState _battery_state;
	float _battery_upper_threshold;
};
#endif