#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>
#include <xsens_driver/xsens_sensor_subscriber.h>
#include <youbot_controller/youbot.h>

// Función que devuelve el equivalente al ángulo
// si se encontrara fuera del rango [0, 2*pi]
double NormalizeAngle(double angle);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "youbot_controller");
    ros::NodeHandle node_handle;
    
    xsens::SensorSubscriberList sensor_subscriber_list(node_handle);
    unsigned int mt_count = sensor_subscriber_list.GetMtCount();
    
    // Si no hay sensores, salimos del programa
    if(mt_count == 0)
    {
        ROS_ERROR("No MTs found. Quitting...");
        return 1;
    }
    ROS_INFO("Detected IMU count: %d",mt_count);
    if(mt_count != 3)
    {
        ROS_WARN("3 sensors are needed by this demo to work properly. " 
            "Please connect 3 Xsens IMUs to the Xbus Master and restart the program. "
            "Quitting...");
        return 1;
    }
    
    // Esperamos a que ROS llame a las funciones callback para
    // leer los datos de los topics
    ROS_INFO("Waiting for the topics to be read...");
    while(sensor_subscriber_list.GetOriQuat(0) == dfv::Quaternion(0.0, 0.0, 0.0, 0.0))
    {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    Youbot youbot(node_handle);
    
    // bucle principal
    while(ros::ok())
    {
        dfv::Quaternion q0 = sensor_subscriber_list.GetOriQuat(0);
        dfv::Quaternion q1 = sensor_subscriber_list.GetOriQuat(1);
        dfv::Quaternion q2 = sensor_subscriber_list.GetOriQuat(2);
        
        dfv::Quaternion q01 = dfv::Quaternion::GetDifference(q0, q1);
        dfv::Quaternion q12 = dfv::Quaternion::GetDifference(q1, q2);
        
        double roll_0;
        double pitch_0;
        double yaw_0;
        
        q0.GetRPY(roll_0, pitch_0, yaw_0);
        /*std::cout << "q0: " << q0 << std::endl;
        std::cout << "roll_0: " << roll_0 << std::endl;
        std::cout << "pitch_0: " << pitch_0 << std::endl;
        std::cout << "yaw_0: " << yaw_0 << std::endl;*/
        
        double roll_01;
        double pitch_01;
        double yaw_01;
        
        q01.GetRPY(roll_01, pitch_01, yaw_01);
        /*std::cout << "q01: " << q01 << std::endl;
        std::cout << "roll_01: " << roll_01 << std::endl;
        std::cout << "pitch_01: " << pitch_01 << std::endl;
        std::cout << "yaw_01: " << yaw_01 << std::endl;*/
        
        double roll_12;
        double pitch_12;
        double yaw_12;
        
        q01.GetRPY(roll_12, pitch_12, yaw_12);
        /*std::cout << "q12: " << q12 << std::endl;
        std::cout << "roll_12: " << roll_12 << std::endl;
        std::cout << "pitch_12: " << pitch_12 << std::endl;
        std::cout << "yaw_12: " << yaw_12 << std::endl;*/
        
        youbot.joint_positions[0] = -NormalizeAngle(yaw_0);
        youbot.joint_positions[1] = -pitch_0 < 0 ? 0 : -pitch_0;
        youbot.joint_positions[2] = -2.5 + 1.0 * (-pitch_01);
        youbot.joint_positions[3] = 1.5 + 1.0 * (-pitch_12);
        youbot.joint_positions[4] = 1.5 + 2.0 * (-roll_01);
        youbot.PublishMessage();
        
        std::cout << "Published angles: " << std::endl;
        for(unsigned int i = 0; i < 5; ++i)
        {
            std::cout << "joint #" << (i+1) << ": " << youbot.joint_positions[i] << std::endl; 
        }
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return 0;
}

double NormalizeAngle(double angle)
{
    while(angle < 0)
    {
        angle += 2*dfv::pi;
    }
    while(angle > 2*dfv::pi)
    {
        angle -= 2*dfv::pi;
    }
    return angle;
}
