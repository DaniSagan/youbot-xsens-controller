/*
 * Programa que toma los datos de los topics
 * de tres sensores xsens, calcula los ángulos
 * de rotación entre cada sensor y los publica 
 * en el topic para mover el brazo robótico del
 * robot Youbot.
 *
 * Autor: Daniel Fernández Villanueva
 * Octubre de 2013
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "youbot_controller_2");
    ros::NodeHandle node_handle;
    /*dfv::Youbot youbot(node_handle);   
    youbot.MovePlatform(1, 0, 0, 1)
          .MovePlatform(0, 1, 0, 1)
          .MovePlatform(-1, 0, 0, 1)
          .MovePlatform(0, -1, 0, 1)
          .StopPlatform();*/
    
    dfv::Youbot youbot(node_handle);
    youbot.base.Enable();
    youbot.arm.Disable();
    
    youbot.base.MoveFor(1, 0, 0, 5)
               .MoveFor(0, 0, 1, 5)
               .MoveFor(1, 0, 0, 5)
               .MoveFor(0, 0, 1, 5)
               .Stop();
               
    std::vector<float> arm_pos_1(5);
    arm_pos_1[0] = 1.5f;
    arm_pos_1[1] = -1.5f;
    arm_pos_1[2] = 1.5f;
    arm_pos_1[3] = -1.5f;
    arm_pos_1[4] = 1.5f;
    
    std::vector<float> arm_pos_2(5);
    arm_pos_2[0] = 0.f;
    arm_pos_2[1] = 0.f;
    arm_pos_2[2] = 0.f;
    arm_pos_2[3] = 0.f;
    arm_pos_2[4] = 0.f;
    
    youbot.arm.SetPos(arm_pos_1).Wait(4.0).SetPos(arm_pos_2).Wait(4.0);
    
    //ros::Duration(1.0).sleep();
    //ros::spinOnce();
    
    return 0;
}

