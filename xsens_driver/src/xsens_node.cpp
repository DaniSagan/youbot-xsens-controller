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
#include <geometry_msgs/QuaternionStamped.h>
#include <dfv/dfv.h>
#include <xsens_driver/utils.h>

int main(int argc, char** argv)
{
    xsens::Driver driver;
    ROS_INFO("Detected sensor count: %d", driver.GetMtCount());
    
    // Asignamos a los sensores una matriz de pre-rotación unitaria
    for(unsigned int i = 0; i < driver.GetMtCount(); ++i)
    {
        driver.SetAlignmentMatrix(i, xsens::DfvToCmtMatrix(dfv::Matrix::Identity(3)));
    }
    
    // Inicializamos el driver. Esto realizará la configuración del sensor 
    // y lo pondrá en modo de medida
    if(driver.Initialize() == false)
    {
        std::cout << "ERROR: No Xsens IMUs found. Quitting..." << std::endl;
        return -1;
    }
    
    std::cout << "Initializing ROS..." << std::endl;
    ros::init(argc, argv, "xsens_node");
    ros::NodeHandle node_handle("~");
    std::cout << "Now publishing data..." << std::endl;
    
    std::vector<ros::NodeHandle> sensor_node_handles(driver.GetMtCount()); 
    for(unsigned int i = 0; i < driver.GetMtCount(); i++)
    {
        std::stringstream ss;
        ss << "sensor" << i;
        sensor_node_handles[i] = ros::NodeHandle(node_handle, ss.str());
    }
    
    
    
    std::vector<ros::Publisher> acc_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> gyr_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> mag_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> raw_acc_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> raw_gyr_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> raw_mag_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> ori_quat_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> ori_mat_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> ori_eul_publishers(driver.GetMtCount());
    
    // Datos calibrados
    for(unsigned int i = 0; i < driver.GetMtCount(); i++)
    {
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_CALIB) != 0)
        {
            acc_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("acc", 1);
            gyr_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("gyr", 1);
            mag_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("mag", 1);
        }
        
        // Datos crudos
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_RAW) != 0)
        {
            raw_acc_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_acc", 1);
            raw_gyr_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_gyr", 1);
            raw_mag_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_mag", 1);
        }
        
        // Datos de orientación
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_ORIENT) != 0)
        {  
            // Cuaternión de orientación
            if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION)
            {
                ori_quat_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::QuaternionStamped>("ori_quat", 1);       
            }
        }
    }
    
    int count = 0;
    
    while(driver.SpinOnce() && ros::ok())
    {
        for(unsigned int i = 0; i < driver.GetMtCount(); i++)
        {
            if((driver.GetOutputMode() & CMT_OUTPUTMODE_CALIB) != 0)
            {
                geometry_msgs::Vector3Stamped msg;
                            
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvVector(driver.GetCalData(i).m_acc));
                msg.header.seq = count;
                acc_publishers[i].publish(msg);
                
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvVector(driver.GetCalData(i).m_gyr));
                msg.header.seq = count;
                gyr_publishers[i].publish(msg);
                
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvVector(driver.GetCalData(i).m_mag));
                msg.header.seq = count;
                mag_publishers[i].publish(msg);
            }
            
            if((driver.GetOutputMode() & CMT_OUTPUTMODE_RAW) != 0)
            {
                geometry_msgs::Vector3Stamped msg;
                            
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvShortVector(driver.GetRawData(i).m_acc));
                msg.header.seq = count;
                raw_acc_publishers[i].publish(msg);
                
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvShortVector(driver.GetRawData(i).m_gyr));
                msg.header.seq = count;
                raw_gyr_publishers[i].publish(msg);
                
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvShortVector(driver.GetRawData(i).m_mag));
                msg.header.seq = count;
                raw_mag_publishers[i].publish(msg);
            }
            
            // Datos de orientación
            if((driver.GetOutputMode() & CMT_OUTPUTMODE_ORIENT) != 0)
            {
                // Cuaternión de orientación
                if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION)
                {
                    geometry_msgs::QuaternionStamped msg;
                    msg.quaternion.w = driver.GetOriQuat(i).m_data[0];
                    msg.quaternion.x = driver.GetOriQuat(i).m_data[1];
                    msg.quaternion.y = driver.GetOriQuat(i).m_data[2];
                    msg.quaternion.z = driver.GetOriQuat(i).m_data[3];
                    msg.header.seq = count;
                    msg.header.stamp = ros::Time::now();
                    ori_quat_publishers[i].publish(msg);       
                }
            }
        }
        
        ++count;
        ros::spinOnce();

    }

    std::cout << "Terminando el programa..." << std::endl;

    return 0;
}
