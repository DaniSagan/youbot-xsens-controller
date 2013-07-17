#ifndef GAZEBO_MODEL_LIST_H
#define GAZEBO_MODEL_LIST_H

#include <list>
#include <sstream>
#include "gazebo_model.h"

namespace gazebo
{
    class CModelList
    {
        public:
            CModelList(ros::NodeHandle& node_handle_);
            ~CModelList();
            
            void AddModel(std::string name, dfv::Vector3 position, std::string xml_model);
            //bool GenerateFromTopic(std::string topic_name);
            //void GetConfig(const test3_xbus::MTConfig::ConstPtr& msg);
            void Spawn();
            //void SubscribeModelToTopic(unsigned int index, std::string topic_name);
            void SetJoint(unsigned int parent_index, unsigned int child_index, dfv::Vector3 parent_joint_position);
            void SetOrientation(unsigned int index, dfv::Quaternion orientation);
            //void EnableGazebo(bool enable);
            //void EnableRviz(bool enable);
            void PublishMessage();
            unsigned int GetCount();
            
        private:
            std::vector<gazebo::CModel*>    model_list;
            ros::NodeHandle&                node_handle;
            ros::ServiceClient              set_state_client;
            ros::ServiceClient              spawn_model_client;
            
    };
};

#endif
