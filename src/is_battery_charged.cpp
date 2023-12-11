#include "bt_amr/is_battery_charged.h"

IsBatteryCharged::IsBatteryCharged(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    _battery_state = sensor_msgs::BatteryState();
    _battery_state.percentage = 1.0;
    _battery_upper_threshold = 0.8;
}

BT::NodeStatus IsBatteryCharged::tick()
{
    _battery_state = *(ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_status", ros::Duration(10.0)));
    ROS_INFO("Battery percentage: %f", _battery_state.percentage);
    return (_battery_state.percentage > _battery_upper_threshold) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::PortsList IsBatteryCharged::providedPorts()
{
    return {};
}
