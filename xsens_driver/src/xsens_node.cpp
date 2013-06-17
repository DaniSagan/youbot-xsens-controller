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
#include <std_msgs/Float64MultiArray.h>
#include <dfv/dfv.h>
#include <xsens_driver/utils.h>

int main(int argc, char** argv)
{
    // Declaración de un objeto driver.
    // Valores por defecto:
    // OutputMode: CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT
    // OutputSettings: CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION
    xsens::Driver driver;
    
    // Aquí podemos cambiar la configuración del sensor
    
    // Número de dispositivos detectados (sin contar el Xbus Master)
    ROS_INFO("Detected sensor count: %d", driver.GetMtCount());
    
    // Asignamos a los sensores una matriz de pre-rotación unitaria
    for(unsigned int i = 0; i < driver.GetMtCount(); ++i)
    {
        driver.SetAlignmentMatrix(i, xsens::DfvToCmtMatrix(dfv::Matrix::Identity(3)));
    }
    
    // Ejemplo para cambiar el modo de salida del sensor
    // para que nos de la matriz de rotación en lugar
    // del cuaternión de orientación:
    
    //driver.SetOutputSettings(CMT_OUTPUTSETTINGS_ORIENTMODE_EULER);
    
    // Inicializamos el driver. Esto realizará la configuración del sensor
    // con los valores que le hayamos asignado hasta ahora 
    // y lo pondrá en modo de medida
    if(driver.Initialize() == false)
    {
        std::cout << "ERROR: No Xsens IMUs found. Quitting..." << std::endl;
        return -1;
    }
    
    // Inicialización de ROS
    ROS_INFO("Initializing ROS...");
    ros::init(argc, argv, "xsens_node");
    ros::NodeHandle node_handle("~");
    
    // Asignamos valor a algunos parámetros
    node_handle.setParam("sensor_count", (int)driver.GetMtCount());
    node_handle.setParam("output_mode", (int)driver.GetOutputMode());
    node_handle.setParam("output_settings", (int)driver.GetOutputSettings());
    
    // Creamos un NodeHandle para cada sensor
    std::vector<ros::NodeHandle> sensor_node_handles(driver.GetMtCount()); 
    for(unsigned int i = 0; i < driver.GetMtCount(); i++)
    {
        std::stringstream ss;
        ss << "sensor" << i;
        sensor_node_handles[i] = ros::NodeHandle(node_handle, ss.str());
    }
    
    
    // Declaramos los publicadores
    std::vector<ros::Publisher> acc_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> gyr_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> mag_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> raw_acc_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> raw_gyr_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> raw_mag_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> ori_quat_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> ori_matrix_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> ori_euler_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> pos_lla_publishers(driver.GetMtCount());
    
    std::vector<ros::Publisher> gps_llh_publishers(driver.GetMtCount());
    std::vector<ros::Publisher> gps_vel_publishers(driver.GetMtCount());
    
    // Creamos los topics a publicar
    for(unsigned int i = 0; i < driver.GetMtCount(); i++)
    {
        // Datos calibrados
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_CALIB) != 0)
        {
            acc_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("acc", 1000);
            gyr_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("gyr", 1000);
            mag_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("mag", 1000);
        }
        
        // Datos crudos
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_RAW) != 0)
        {
            raw_acc_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_acc", 1000);
            raw_gyr_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_gyr", 1000);
            raw_mag_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("raw_mag", 1000);
        }
        
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_POSITION) != 0)
        {
            pos_lla_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("pos_lla", 1000);
        }
        
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_GPSPVT_PRESSURE) != 0)
        {
            gps_llh_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("gps_llh", 1000);
            gps_vel_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::Vector3Stamped>("gps_vel", 1000);
        }
        
        // Datos de orientación
        if((driver.GetOutputMode() & CMT_OUTPUTMODE_ORIENT) != 0)
        {  
            // Cuaternión de orientación
            if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION)
            {
                ori_quat_publishers[i] = sensor_node_handles[i].advertise<geometry_msgs::QuaternionStamped>("ori_quat", 1000);       
            }
            
            // Matriz de orientación
            if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX)
            {
                ori_matrix_publishers[i] = sensor_node_handles[i].advertise<std_msgs::Float64MultiArray>("ori_matrix", 1000);       
            }
            
            // Ángulos de Euler
            if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_EULER)
            {
                ori_euler_publishers[i] = sensor_node_handles[i].advertise<std_msgs::Float64MultiArray>("ori_euler", 1000);       
            }
        } 
    }
    
    // Contador
    int count = 0;
    
    // Empezamos a publicar los datos
    ROS_INFO("Now publishing data...");
    
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
            
            if((driver.GetOutputMode() & CMT_OUTPUTMODE_POSITION) != 0)
            {
                geometry_msgs::Vector3Stamped msg;
                            
                msg = xsens::ToVector3StampedMsg(xsens::CmtToDfvVector(driver.GetPositionLLA(i)));
                msg.header.seq = count;
                pos_lla_publishers[i].publish(msg);
            }
            
            if((driver.GetOutputMode() & CMT_OUTPUTMODE_GPSPVT_PRESSURE) != 0)
            {
                CmtGpsPvtData data = driver.GetGpsPvtData(i);
                
                geometry_msgs::Vector3Stamped llh_msg;
                llh_msg.vector.x = (float)data.m_latitude;
                llh_msg.vector.y = (float)data.m_longitude;
                llh_msg.vector.z = (float)data.m_height;
                llh_msg.header.stamp = ros::Time::now();
                llh_msg.header.seq = count;
                gps_llh_publishers[i].publish(llh_msg);
                
                geometry_msgs::Vector3Stamped vel_msg;
                vel_msg.vector.x = (float)data.m_veln;
                vel_msg.vector.y = (float)data.m_vele;
                vel_msg.vector.z = (float)data.m_veld;
                vel_msg.header.stamp = ros::Time::now();
                vel_msg.header.seq = count;
                gps_vel_publishers[i].publish(vel_msg);
                
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
                
                // Matriz de orientación
                if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX)
                {
                    std_msgs::Float64MultiArray msg;
                    msg.data.clear();
                    msg.data.resize(9);
                    msg.data[0] = driver.GetOriMatrix(i).m_data[0][0];
                    msg.data[1] = driver.GetOriMatrix(i).m_data[0][1];
                    msg.data[2] = driver.GetOriMatrix(i).m_data[0][2];
                    msg.data[3] = driver.GetOriMatrix(i).m_data[1][0];
                    msg.data[4] = driver.GetOriMatrix(i).m_data[1][1];
                    msg.data[5] = driver.GetOriMatrix(i).m_data[1][2];
                    msg.data[6] = driver.GetOriMatrix(i).m_data[2][0];
                    msg.data[7] = driver.GetOriMatrix(i).m_data[2][1];
                    msg.data[8] = driver.GetOriMatrix(i).m_data[2][2];
                    ori_matrix_publishers[i].publish(msg);       
                }
                
                // Ángulos de Euler
                if((driver.GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) == CMT_OUTPUTSETTINGS_ORIENTMODE_EULER)
                {
                    std_msgs::Float64MultiArray msg;
                    msg.data.clear();
                    msg.data.resize(3);
                    msg.data[0] = driver.GetOriEuler(i).m_roll;
                    msg.data[1] = driver.GetOriEuler(i).m_pitch;
                    msg.data[2] = driver.GetOriEuler(i).m_yaw;
                    ori_euler_publishers[i].publish(msg);       
                }
            }
        }
        
        ++count;
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
        

    }

    ROS_INFO("Finishing program...");

    return 0;
}
