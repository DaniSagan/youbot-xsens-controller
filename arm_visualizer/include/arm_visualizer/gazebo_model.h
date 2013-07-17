#ifndef GAZEBO_MODEL_H
#define GAZEBO_MODEL_H

#include <ros/ros.h>
#include <dfv/dfv.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <urdf/model.h>
#include <fstream>

namespace gazebo
{
    class CModel
    {
        public:
            CModel(ros::NodeHandle& node_handle_, 
                   ros::ServiceClient& set_state_client_, 
                   ros::ServiceClient& spawn_model_client_);
            ~CModel();
            
            //bool                SubscribeToTopic(std::string topic_name);
            //void                SetModelState(const test3_xbus::MTQuaternion::ConstPtr& msg);
            void                Spawn();
            bool                ReadUrdf(std::string filename);
            
            void                SetName(std::string name);
            void                SetPosition(dfv::Vector3 position);
            void                SetOrientation(dfv::Quaternion orientation);
            void                PublishMessage();
            void                SetParent(gazebo::CModel* lp_parent, dfv::Vector3 parent_joint_position); 
            dfv::Vector3        GetPosition();  
            dfv::Quaternion     GetOrientation();
            
            void                SetId(int id);
            int                 GetId();
            
            static int          count;
            
        private:
            std::string         name;       
            dfv::Quaternion     orientation;
            dfv::Vector3        position;           
            ros::NodeHandle&    node_handle;
            
            ros::ServiceClient& set_state_client;
            ros::ServiceClient& spawn_model_client;
            
            std::string         urdf_model;
            CModel*             lp_parent;
            dfv::Vector3        parent_joint_position;
            
            int                 id;
            
    };
};

#endif
