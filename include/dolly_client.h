#ifndef DOLLY_CLIENT_H
#define DOLLY_CLIENT_H

#include <dolly_pose_estimation/DollyPose.h>

using std::string;

class FindDolly : public BT::AsyncActionNode 
{
public: 
	FindDolly(const string& name, const BT::NodeConfiguration& config) 
    : BT::AsyncActionNode(name, config)
	{
        
        _client = _nh.serviceClient<dolly_pose_estimation::DollyPose>("/dolly_pose_estimation");

    }

    BT::NodeStatus tick() override
	{   _scan_data = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", _nh));
        _srv.request.scan_data = _scan_data;
        if (_client.call(_srv)) 
        {
            std::vector<geometry_msgs::Pose> dolly_positions = _srv.response.poses.poses;

            // for (size_t i = 0; i < dolly_positions.size(); ++i) {
            //     geometry_msgs::Pose pose = dolly_positions[i];
            //     ROS_INFO("Dolly %zu: X = %f, Y = %f, Yaw = %f", i, pose.position.x, pose.position.y, pose.orientation.z); 
            // }
            geometry_msgs::Pose pose = dolly_positions[0];
            Pose3D pose_goal = {pose.position.x, pose.position.y, pose.orientation.z};
            setOutput<Pose3D>("goal",pose_goal);
            return BT::NodeStatus::SUCCESS;
        } 
        
        else 
        {
            ROS_ERROR("Service call failed");
            return BT::NodeStatus::FAILURE;
        }
        
	}

    static PortsList providedPorts()
    {
      return {OutputPort<Pose3D>("goal")};
    }

private:
	ros::NodeHandle _nh;
    ros::ServiceClient _client;
	sensor_msgs::LaserScan _scan_data;
    dolly_pose_estimation::DollyPose _srv;
};
#endif 