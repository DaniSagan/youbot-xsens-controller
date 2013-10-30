/*
 * Programa que toma los datos de los topics
 * de tres sensores xsens, calcula los ángulos
 * de rotación entre cada sensor y los publica 
 * en el topic para mover el brazo robótico del
 * robot Youbot.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>
#include "youbot_controller/arm_controller.h"
#include "youbot_controller/base_controller.h"

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "youbot_controller");
    ros::NodeHandle node_handle;    
    dfv::XsensListener sensors(node_handle);
    
    // Si no hay 4 sensores, salimos del programa
    if(sensors.Count() != 4)
    {
        ROS_WARN("4 sensors are needed for this demo to work properly. " 
            "Please connect 4 Xsens IMUs to the Xbus Master and restart the program. "
            "Quitting...");
        return 1;
    }
    
    // Esperamos a que ROS llame a las funciones callback para
    // leer los datos de los topics
    ROS_INFO("Waiting for the topics to be read...");
    while(sensors.GetOriQuat(0) == dfv::Quaternion(0.0, 0.0, 0.0, 0.0))
    {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Creating youbot...");
    dfv::Youbot youbot(node_handle);
    youbot.arm.Disable();
    youbot.base.Enable();    
    ROS_INFO("Youbot Created");
    
    ArmController arm_controller(sensors, youbot);
    BaseController base_controller(sensors, youbot);
    
    ROS_INFO("Calibrating Sensor Orientation...");
    arm_controller.OnInit(); // calibración de orientación de los sensores
    ROS_INFO("Calibration Done.");
    base_controller.OnInit(); // No hace nada
    
    // bucle principal
    while(ros::ok())
    {
        arm_controller.OnUpdate();
        base_controller.OnUpdate();        
        youbot.Wait(0.05f);
        ros::spinOnce();
    }
    
    return 0;
}
