#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>
#include <xsens_driver/xsens_sensor_subscriber.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "youbot_controller");
    ros::NodeHandle node_handle;
    
    xsens::SensorSubscriberList sensor_subscriber_list(node_handle);
    unsigned int mt_count = sensor_subscriber_list.GetMtCount();
    
    if(mt_count == 0)
    {
        ROS_ERROR("No MTs found. Quitting...");
        return 1;
    }
    ROS_INFO("Detected IMU count: %d",mt_count);
    ros::spinOnce();
    
    while(ros::ok())
    {
        dfv::Quaternion q0 = sensor_subscriber_list.GetOriQuat(0);
        std::cout << "q0: " << q0 << std::endl;
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return 0;
}
