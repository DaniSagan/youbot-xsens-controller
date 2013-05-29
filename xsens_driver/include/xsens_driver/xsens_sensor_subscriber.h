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
            SensorSubscriber(unsigned int mt_index_ = 0);
            ~SensorSubscriber();
            
            bool                    SubscribeToTopics();
            
            const dfv::Vector3      GetAcc() const;
            const dfv::Vector3      GetGyr() const;
            const dfv::Vector3      GetMag() const;
            
            const dfv::Quaternion   GetOriQuat() const;
            const dfv::Matrix       GetOriMatrix() const;
            const dfv::Vector3      GetOriEuler() const;
            
        private:
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
            
            ros::NodeHandle     node_handle;
            
            ros::Subscriber     acc_subscriber;
            void                AccSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            
            ros::Subscriber     gyr_subscriber;
            void                GyrSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            
            ros::Subscriber     mag_subscriber;
            void                MagSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            
            ros::Subscriber     ori_quat_subscriber;
            void                OriQuatSubCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
            
            ros::Subscriber     ori_matrix_subscriber;
            void                OriMatrixSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            
            ros::Subscriber     ori_euler_subscriber;
            void                OriEulerSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            
    };
    
    class SensorSubscriberList
    {
        public:
            SensorSubscriberList();
            ~SensorSubscriberList(); 
            
        private:
            std::vector<SensorSubscriber> sensors;
            ros::NodeHandle node_handle;
            
    };
}

#endif
