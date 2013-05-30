#include <youbot_controller/youbot.h>

Youbot::Youbot(ros::NodeHandle& node_handle_, std::string topic_name_, std::string gripper_topic_name):
    node_handle(node_handle_)
{
    this->publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(topic_name_, 1000);
    this->gripper_publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(gripper_topic_name, 1000);    
}

Youbot::~Youbot()
{
    
}

void Youbot::PublishMessage()
{
    brics_actuator::JointPositions msg;
    std::vector<brics_actuator::JointValue> v_joint_values(5);
    
    for(int i = 0; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "arm_joint_" << (i+1);
        v_joint_values[i].joint_uri = ss.str();
        v_joint_values[i].unit = std::string("rad");
        v_joint_values[i].value = this->joint_positions[i];
    }
    
    msg.positions = v_joint_values;    
    this->publisher.publish(msg);
    
    brics_actuator::JointPositions gripper_msg;
    std::vector<brics_actuator::JointValue> v_gripper_values(2);
    
    v_gripper_values[0].joint_uri = "gripper_finger_joint_l";
    v_gripper_values[0].unit = std::string("m");
    v_gripper_values[0].value = this->gripper_positions[0];
    
    v_gripper_values[1].joint_uri = "gripper_finger_joint_r";
    v_gripper_values[1].unit = std::string("m");
    v_gripper_values[1].value = this->gripper_positions[1];
    
    gripper_msg.positions = v_gripper_values;
    this->gripper_publisher.publish(gripper_msg);

}
