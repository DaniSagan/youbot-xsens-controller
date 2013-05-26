/*
 * Programa que publica en topics de ROS
 * los datos obtenidos de los sensores Xsens
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#include <iostream>
#include <cmath>
#include <xsens_driver/xsens_driver.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

int main(int argc, char** argv)
{
    xsens::Driver driver;
    if(driver.Initialize() == false)
    {
        std::cout << "ERROR: No se han encontrado sensores Xsens. Abortando..." << std::endl;
        return -1;
    }
    
    std::cout << "Inicializando ROS..." << std::endl;
    ros::init(argc, argv, "xsens_node");
    ros::NodeHandle node_handle("~");
    std::cout << "Publicando datos..." << std::endl;
    
    ros::Publisher acc_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("acc", 1);
    
    int count = 0;
    
    while(driver.SpinOnce() && ros::ok())
    {
        CmtCalData cal_data = driver.GetCalData();
        geometry_msgs::Vector3Stamped msg;
        msg.header.seq = count;
        msg.header.stamp = ros::Time::now();
        msg.vector.x = cal_data.m_acc.m_data[0];
        msg.vector.y = cal_data.m_acc.m_data[1];
        msg.vector.z = cal_data.m_acc.m_data[2];
        acc_pub.publish(msg);
        ++count;
        ros::spinOnce();

    }

    std::cout << "Terminando el programa..." << std::endl;

    return 0;
}
