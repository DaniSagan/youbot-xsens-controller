#include "arm_visualizer/gazebo_model_list.h"

namespace gazebo
{

    CModelList::CModelList(ros::NodeHandle& node_handle_):
        node_handle(node_handle_)
    {
        this->set_state_client = node_handle.serviceClient<gazebo_msgs::SetModelState> ("/gazebo/set_model_state");
        this->spawn_model_client = node_handle.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
    }

    CModelList::~CModelList()
    {
    }

    void CModelList::AddModel(std::string name, dfv::Vector3 position, std::string xml_model)
    {
        gazebo::CModel* lp_model = new gazebo::CModel(this->node_handle, 
                                                      this->set_state_client, 
                                                      this->spawn_model_client);
        lp_model->SetName(name);    
        lp_model->SetPosition(position);
        lp_model->ReadUrdf(xml_model);
        this->model_list.push_back(lp_model);
    }

    void CModelList::Spawn()
    {
        for(unsigned int i = 0; i < this->model_list.size(); i++)
        {
            this->model_list[i]->Spawn();
        }
    }

    void CModelList::SetJoint(unsigned int parent_index, 
                                      unsigned int child_index,
                                      dfv::Vector3 parent_joint_position)
    {
        this->model_list[child_index]->SetParent(this->model_list[parent_index], parent_joint_position);
    }

    void CModelList::SetOrientation(unsigned int index, dfv::Quaternion orientation)
    {
        this->model_list[index]->SetOrientation(orientation);
    }

    void CModelList::PublishMessage()
    {
        for(unsigned int i = 0; i < this->model_list.size(); i++)
        {
            this->model_list[i]->PublishMessage();
        }
    }

    unsigned int CModelList::GetCount()
    {
        return this->model_list.size();
    }

}
