/*
 * Clase Youbot. Encapsula el mecanismo
 * de publicación de los mensajes necesarios
 * para mover las articulaciones del
 * robot Youbot.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#ifndef YOUBOT_H
#define YOUBOT_H

#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <dfv/dfv.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

//! El espacio de nombres para la librería dfv
namespace dfv
{

class Gripper
{
public:
    Gripper(ros::NodeHandle& _node_handle);
    ~Gripper();
    
    enum State
    {
        open = 0,
        closed
    };
    
    void Open();
    void Close();
    
private:
    static const std::string topic; 
    static const float open_pos;
    static const float closed_pos;
    ros::NodeHandle& node_handle;
    State state;
    std::vector<brics_actuator::JointValue> values;
    ros::Publisher publisher;
    
    void Publish();       
};

class Joint
{
public:
    Joint(float _min_pos, float _max_pos, float _reset_pos);
    ~Joint();   
     
    void SetTarget(float target_pos);
    void SetTargetRel(float target_pos);
    float GetState() const;
    
    friend class Arm;
    
private:    
    void SetState(float state_pos);
    
    float min_pos;
    float max_pos;
    float reset_pos;
    float state;    
    float target;    
};

class Arm
{
public:
    Arm(ros::NodeHandle& _node_handle);
    ~Arm();
    
    std::vector<Joint> joints;
    Arm& SetPos(std::vector<float> joint_pos);
    Arm& Wait(float time);
    Arm& Enable();
    Arm& Disable();
private:
    ros::NodeHandle& node_handle;
    static const std::string command_topic;
    static const std::string state_topic;
    ros::Publisher command_publisher;
    ros::Subscriber state_subscriber; 
    std::vector<brics_actuator::JointValue> joint_values;   
    static const float joint_min_pos[];
    static const float joint_max_pos[];
    static const float joint_offset[];
    bool enabled;
    
    void StateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void Publish();    
};

class Base
{
public:
    Base(ros::NodeHandle& _node_handle);
    ~Base();   
    
    Base& Move(float linear_vel, float side_vel, float angular_vel);
    Base& MoveFor(float linear_vel, float side_vel, float angular_vel, float time);
    Base& Stop();
    Base& Enable();
    Base& Disable();
    
private:
    ros::NodeHandle& node_handle;
    ros::Publisher publisher;  
    static const std::string topic;
    float linear_vel;
    float side_vel;
    float angular_vel;
    bool enabled;       
      
    void Publish();
};

class Youbot
{
public:
    Youbot(ros::NodeHandle& node_handle_);
    ~Youbot();
    
    Arm arm;
    Base base;
    Gripper gripper;
    
    Youbot& Wait(float time);
    
private:
    ros::NodeHandle& node_handle;
};

/*! \brief Clase que encapsula la comunicaciones de ROS para el control del robot YouBot
*/

/*
class Youbot
{
    public:
        Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_ = "arm_1/arm_controller/position_command", 
               std::string gripper_topic_name_ = "arm_1/gripper_controller/position_command",
               std::string joint_states_topic_name = "joint_states",
               std::string cmd_vel_topic_name = "cmd_vel");
        ~Youbot();
        
        double joint_positions[5];
        double gripper_positions[2];
        
        dfv::Vector3 linear_vel;
        dfv::Vector3 angular_vel;
        
        void PublishArmPosition();
        void PublishPlatformVel();
        
        dfv::Quaternion GetJointLocalQuaternion(int index) const;
        dfv::Quaternion GetJointLocalQuatFromAngle(int index, float angle) const;
        
        dfv::Vector3 GetJointPosition(int index) const;
        dfv::Vector3 GetJointPosFromAngles(unsigned int index, 
                                           const std::vector<float>& joint_angles) const;
        
        void OpenGripper();
        void CloseGripper();
        void ResetArmPosition();
        
        std::vector<float> FindAnglesForPos(dfv::Vector3& target_pos);
        
        static const float joint_min_pos[];
        static const float joint_max_pos[];
        static const float joint_ini_pos[];
        
        static const dfv::Vector3 r[];
        
        enum GripperState
        {
            closed = 0,
            open
        };
        
        std::vector<float> joint_states;
        
        Youbot& MovePlatform(double linear_vel, double side_vel, double angular_vel, double time);
        Youbot& StopPlatform();
        
    protected:
        ros::NodeHandle& node_handle;
        ros::Publisher arm_publisher;
        ros::Publisher gripper_publisher;
        ros::Publisher cmd_vel_publisher;
        
        std::vector<brics_actuator::JointValue> v_joint_values;
        std::vector<brics_actuator::JointValue> v_gripper_values;
        
        GripperState gripper_state;
        
        ros::Subscriber joint_states_subscriber;
        void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

};*/

}

#endif
