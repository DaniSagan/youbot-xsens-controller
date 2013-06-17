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
#include <xsens_driver/xsens_sensor_subscriber.h>
#include <youbot_controller/youbot.h>

// Función que devuelve el equivalente al ángulo
// dentro del rango [0, 2*pi]
double NormalizeAngle(double angle);
void CorrectRPY(double& roll, double& pitch, double& yaw);

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
        
        // Cálculo de los ángulos entre cada sensor
        
        double roll_0;
        double pitch_0;
        double yaw_0;        
        q0.GetRPY(roll_0, pitch_0, yaw_0);
        CorrectRPY(roll_0, pitch_0, yaw_0);
        
        double roll_01;
        double pitch_01;
        double yaw_01;        
        q01.GetRPY(roll_01, pitch_01, yaw_01);
        CorrectRPY(roll_01, pitch_01, yaw_01);
        
        double roll_12;
        double pitch_12;
        double yaw_12;        
        q12.GetRPY(roll_12, pitch_12, yaw_12);
        CorrectRPY(roll_12, pitch_12, yaw_12);
        
        // Adaptación de los ángulos obtenidos
        // teniendo en cuenta los offsets de cada
        // articulación
        
        double angs[5];
        angs[0] = NormalizeAngle(-yaw_0);
        angs[1] = -pitch_0 < 0 ? 0 : -pitch_0;
        angs[2] = -2.5 + 1.0 * (-pitch_01);
        angs[3] = 1.5 + 1.0 * (-pitch_12);
        angs[4] = 1.5 + 2.0 * (-roll_01);
                
        // Pasamos los ángulos al robot
        // Los limitamos al intervalo que acepta cada articulación
        
        youbot.joint_positions[0] = (angs[0] <  0.02) ?  0.02 : ((angs[0] >  5.83) ?  5.83 : angs[0]);
        youbot.joint_positions[1] = (angs[1] <  0.02) ?  0.02 : ((angs[1] >  2.60) ?  2.60 : angs[1]);
        youbot.joint_positions[2] = (angs[2] < -5.01) ? -5.01 : ((angs[2] > -0.02) ? -0.02 : angs[2]);
        youbot.joint_positions[3] = (angs[3] <  0.03) ?  0.03 : ((angs[3] >  3.41) ?  3.41 : angs[3]);
        youbot.joint_positions[4] = (angs[4] <  0.12) ?  0.12 : ((angs[4] >  5.63) ?  5.63 : angs[4]);
        
        // Publicamos en el topic del robot
        
        youbot.PublishMessage();
        
        // Imprimimos en pantalla los ángulos que le hemos pasado al robot
        
        std::cout << "Published angles: " << std::endl;
        for(unsigned int i = 0; i < 5; ++i)
        {
            std::cout << "joint #" << (i+1) << ": " << youbot.joint_positions[i] << std::endl; 
        }
        std::cout << "--------------------------------" << std::endl;
        
        // Frecuencia del bucle: 20 Hz
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

void CorrectRPY(double& roll, double& pitch, double& yaw)
{
    if (roll > dfv::pi/2.0)
    {
        roll -= dfv::pi;
        pitch = (pitch < 0)? - dfv::pi - pitch : dfv::pi - pitch;
        yaw = (yaw < 0)? yaw + dfv::pi : yaw - dfv::pi;
        
    }
    else if (roll < -dfv::pi/2.0)
    {
        roll += dfv::pi;
        pitch = (pitch < 0)? - dfv::pi - pitch : dfv::pi - pitch;
        yaw = (yaw < 0)? yaw + dfv::pi : yaw - dfv::pi;
    }
}
