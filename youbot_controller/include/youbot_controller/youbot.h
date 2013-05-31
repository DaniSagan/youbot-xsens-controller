#ifndef YOUBOT_H
#define YOUBOT_H

#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>

class Youbot
{
    public:
        Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_ = "arm_1/arm_controller/position_command", 
               std::string gripper_topic_name_ = "arm_1/gripper_controller/position_command");
        ~Youbot();
        
        double joint_positions[5];
        double gripper_positions[2];
        void PublishMessage(bool publish_gripper = false);
        
    private:
        ros::NodeHandle& node_handle;
        ros::Publisher arm_publisher;
        ros::Publisher gripper_publisher;
        
        std::vector<brics_actuator::JointValue> v_joint_values;
        std::vector<brics_actuator::JointValue> v_gripper_values;
};

#endif