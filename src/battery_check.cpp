#include "bt_amr/battery_check.h"

BatteryCheck::BatteryCheck(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
    _battery_state = sensor_msgs::BatteryState();
    _battery_state.percentage = 1.0;
    _battery_lower_threshold = 0.2;
}

BT::NodeStatus BatteryCheck::tick()
{
    _battery_state = *(ros::topic::waitForMessage<sensor_msgs::BatteryState>("/battery_status", ros::Duration(10.0)));
    ROS_INFO("Battery percentage: %f", _battery_state.percentage);
    return (_battery_state.percentage < _battery_lower_threshold) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::PortsList BatteryCheck::providedPorts()
{
    return {};
}
