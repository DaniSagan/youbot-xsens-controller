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

// ángulos límite
const double ang_min[] = {0.010, 0.010, -5.026, 0.022, 0.110};
const double ang_max[] = {5.840, 2.617, -0.015, 3.429, 5.641};
const double sec_ang = 0.05;

std::vector<double> GetAngles(xsens::SensorSubscriberList& sensors);

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
    
    std::vector<double> angs;
    
    unsigned int c = 200;
    double ang_0_offset = 0;
    ROS_INFO("Calibrating Sensor Orientation...");
    
    while(ros::ok() && c != 0)
    {
        angs = GetAngles(sensor_subscriber_list);
        ang_0_offset = (ang_min[0] + ang_max[0]) / 2.0 - angs[0];
        ros::Duration(0.05).sleep();
        c--;
    }
    ROS_INFO("Calibration Done.");
    
    // bucle principal
    while(ros::ok())
    {
        // Obtención de los ángulos a partir de los cuaterniones de los sensores
        angs = GetAngles(sensor_subscriber_list);
        
        angs[0] += ang_0_offset;
        angs[0] = NormalizeAngle(angs[0]);
        
        // Corrección de los ángulos para que no superen los límites
        for(int i = 0; i < 5; i++)
        {
            youbot.joint_positions[i] = (angs[i] <  ang_min[i] + sec_ang) ?  
                ang_min[i] + sec_ang : ((angs[i] >  ang_max[i] - sec_ang) ?  
                    ang_max[i] - sec_ang : angs[i]);
                        
        }
        
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

std::vector<double> GetAngles(xsens::SensorSubscriberList& sensors)
{
    std::vector<double> angles(5);
    
    dfv::Quaternion q0 = sensors.GetOriQuat(0);
    dfv::Quaternion q1 = sensors.GetOriQuat(1);
    dfv::Quaternion q2 = sensors.GetOriQuat(2);
    
    dfv::Quaternion q0p = dfv::Quaternion::GetRotationQuaternion(dfv::Vector3::i.GetRotated(q0), dfv::pi / 4.0);
    dfv::Quaternion q1p = dfv::Quaternion::GetRotationQuaternion(dfv::Vector3::i.GetRotated(q1), dfv::pi / 4.0);
    dfv::Quaternion q2p = dfv::Quaternion::GetRotationQuaternion(dfv::Vector3::i.GetRotated(q2), dfv::pi / 4.0);
    
    //dfv::Quaternion q01 = dfv::Quaternion::GetDifference(q0p*q0, q1p*q1);
    dfv::Quaternion q01 = dfv::Quaternion::GetDifference(q0, q1);
    dfv::Quaternion q12 = dfv::Quaternion::GetDifference(q1p*q1, q2p*q2);
    
    // Cálculo de los ángulos entre cada sensor
    
    double roll_0;
    double pitch_0;
    double yaw_0;        
    q0.GetRPY(roll_0, pitch_0, yaw_0, 1);
    if(fabs(roll_0) > dfv::pi/2.0)
    {
        q0.GetRPY(roll_0, pitch_0, yaw_0, 2);
    }
    
    double roll_01;
    double pitch_01;
    double yaw_01;        
    q01.GetRPY(roll_01, pitch_01, yaw_01, 1);
    if(fabs(roll_01) > dfv::pi/2.0)
    {
        q01.GetRPY(roll_01, pitch_01, yaw_01, 2);
    }
    
    double roll_12;
    double pitch_12;
    double yaw_12;        
    q12.GetRPY(roll_12, pitch_12, yaw_12, 1);
    if(fabs(roll_12) > dfv::pi/2.0)
    {
        q12.GetRPY(roll_12, pitch_12, yaw_12, 2);
    }
    
    // Adaptación de los ángulos obtenidos
    // teniendo en cuenta los offsets de cada
    // articulación
    
    double offsets[] = {2.9, -0.4, -2.7, 1.5, 1.5};
    
    double angs[5];
    angles[0] = offsets[0] + NormalizeAngle(-yaw_0);
    angles[1] = offsets[1] + 1.0 * (-pitch_0);
    //angles[2] = offsets[2] + 1.0 * (-yaw_01);
    angles[2] = offsets[2] + 1.0 * (-pitch_01);
    angles[3] = offsets[3] + 1.0 * (yaw_12);
    //angles[4] = offsets[4] ;//+ 2.0 * (-roll_01);
    angles[4] = 2.91237f;
    
    return angles;
}
