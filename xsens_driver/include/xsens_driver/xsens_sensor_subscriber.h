/*
 * Clases SensorSubscriber y SensorSubscriberList
 * 
 * Estas clases no son utilizadas por el driver.
 * Forman parte de la librería xsens_driver, que
 * proporciona una interfaz sencilla para acceder
 * a los datos publicados en ROS por el driver en
 * otros programas.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo 2013
 *
 */

#ifndef XSENS_SENSOR_SUBSCRIBER_H
#define XSENS_SENSOR_SUBSCRIBER_H

#include <dfv/dfv.h>
#include <sstream>
#include <xsens_driver/cmtdef.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace xsens
{
    class SensorSubscriber
    {
        public:
            SensorSubscriber(unsigned int mt_index_, ros::NodeHandle& node_handle_);
            ~SensorSubscriber();
            
            bool                    SubscribeToTopics();
            
            // Función que devuelve el vector aceleración
            const dfv::Vector3      GetAcc() const;
            
            // Función que devuelve el vector giróscopo
            const dfv::Vector3      GetGyr() const;
            
            // Función que devuelve el vector campo magnético
            const dfv::Vector3      GetMag() const;
            
            // Función que devuelve el cuaternión de orientación
            const dfv::Quaternion   GetOriQuat() const;
            
            // Función que devuelve la matriz de orientación
            const dfv::Matrix       GetOriMatrix() const;
            
            // Función que devuelve un vector con los ángulos de Euler
            const dfv::Vector3      GetOriEuler() const;
            
            
            
        private:
            ros::NodeHandle&    node_handle;
            unsigned int        mt_index;
            
            std::string         acc_topic_name;
            std::string         gyr_topic_name;
            std::string         mag_topic_name;
            std::string         ori_quat_topic_name;
            std::string         ori_matrix_topic_name;
            std::string         ori_euler_topic_name;
            
            CmtOutputMode       output_mode;
            CmtOutputSettings   output_settings;
            
            dfv::Vector3        acc;
            dfv::Vector3        gyr;
            dfv::Vector3        mag;
            
            dfv::Quaternion     ori_quat;
            dfv::Matrix         ori_matrix;
            dfv::Vector3        ori_euler;
            
            dfv::Vector3        position_lla;            
            double              temperature;
            
            ros::Subscriber     acc_subscriber;
            ros::Subscriber     gyr_subscriber;
            ros::Subscriber     mag_subscriber;
            ros::Subscriber     ori_quat_subscriber;
            ros::Subscriber     ori_matrix_subscriber;
            ros::Subscriber     ori_euler_subscriber;
            
            void                AccSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                GyrSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                MagSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                OriQuatSubCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
            void                OriMatrixSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void                OriEulerSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    };
    
    class SensorSubscriberList
    {
        public:
            SensorSubscriberList(ros::NodeHandle& node_handle_);
            ~SensorSubscriberList(); 

            // Función que devuelve el número de sensores detectados
            unsigned int GetMtCount() const;
            
            // Función que devuelve el vector aceleración
            const dfv::Vector3      GetAcc(unsigned int mt_index) const;
            
            // Función que devuelve el vector giróscopo
            const dfv::Vector3      GetGyr(unsigned int mt_index) const;
            
            // Función que devuelve el vector campo magnético
            const dfv::Vector3      GetMag(unsigned int mt_index) const;
            
            // Función que devuelve el cuaternión de orientación
            const dfv::Quaternion   GetOriQuat(unsigned int mt_index) const;
            
            // Función que devuelve la matriz de orientación
            const dfv::Matrix       GetOriMatrix(unsigned int mt_index) const;
            
            // Función que devuelve un vector con los ángulos de Euler
            const dfv::Vector3      GetOriEuler(unsigned int mt_index) const;
            
        private:            
            ros::NodeHandle node_handle;
            unsigned int mt_count;
            SensorSubscriber** sensors;
            
    };
}

#endif
