#include "arm_visualizer/gazebo_model.h"

namespace gazebo
{

    int gazebo::CModel::count = 0;

    CModel::CModel(ros::NodeHandle& node_handle_,
                           ros::ServiceClient& set_state_client_,        
                           ros::ServiceClient& spawn_model_client_) : 
        name("unnamed"),
        node_handle(node_handle_),
        set_state_client(set_state_client_),
        spawn_model_client(spawn_model_client_),
        lp_parent(NULL)    
    {
        this->SetId(gazebo::CModel::count);
        gazebo::CModel::count++;
    }

    CModel::~CModel()
    {

    }

    void CModel::Spawn()
    {
        gazebo_msgs::SpawnModel spawn_model_msg;
        spawn_model_msg.request.model_name                  = this->name;
        spawn_model_msg.request.model_xml                   = this->urdf_model;
        spawn_model_msg.request.initial_pose.position.x     = this->position.x;
        spawn_model_msg.request.initial_pose.position.y     = this->position.y;
        spawn_model_msg.request.initial_pose.position.z     = this->position.z;
        spawn_model_msg.request.initial_pose.orientation.w  = 0.0;
        spawn_model_msg.request.initial_pose.orientation.x  = 0.0;
        spawn_model_msg.request.initial_pose.orientation.y  = 0.0;
        spawn_model_msg.request.initial_pose.orientation.z  = 0.0;
        spawn_model_msg.request.reference_frame             = "world";
        
        this->spawn_model_client.call(spawn_model_msg);    
        
    }

    bool CModel::ReadUrdf(std::string filename)
    {
        std::fstream xml_file(filename.c_str(), std::fstream::in);
        std::string xml_string;
        if (xml_file.is_open())
        {
            while ( xml_file.good() )
            {
                std::string line;
                std::getline( xml_file, line);
                xml_string += (line + "\n");
            }
            xml_file.close();
            this->urdf_model = xml_string;
            return true;
        }
        else
        {
            std::cout << "Could not open file [" << filename << "] for parsing." << std::endl;
            return false;
        }
    }

    void CModel::SetName(std::string name)
    {
        this->name = name;
    }

    void CModel::SetPosition(dfv::Vector3 position)
    {
        this->position = position;
    }

    void CModel::SetOrientation(dfv::Quaternion orientation)
    {
        this->orientation = orientation;
    }

    void CModel::PublishMessage()
    {
        gazebo_msgs::SetModelState set_state_msg;
            
        set_state_msg.request.model_state.model_name            = this->name;
        set_state_msg.request.model_state.reference_frame       = "world";
        set_state_msg.request.model_state.pose.position.x       = this->GetPosition().x;
        set_state_msg.request.model_state.pose.position.y       = this->GetPosition().y;
        set_state_msg.request.model_state.pose.position.z       = this->GetPosition().z;
        set_state_msg.request.model_state.pose.orientation.w    = this->orientation.w;
        set_state_msg.request.model_state.pose.orientation.x    = this->orientation.x;
        set_state_msg.request.model_state.pose.orientation.y    = this->orientation.y;
        set_state_msg.request.model_state.pose.orientation.z    = this->orientation.z;
        set_state_msg.request.model_state.twist.linear.x        = 0.0;
        set_state_msg.request.model_state.twist.linear.y        = 0.0;
        set_state_msg.request.model_state.twist.linear.z        = 0.0;
        set_state_msg.request.model_state.twist.angular.x       = 0.0;
        set_state_msg.request.model_state.twist.angular.y       = 0.0;
        set_state_msg.request.model_state.twist.angular.z       = 0.0;
        
        if(!this->set_state_client.call(set_state_msg))
        {
            ROS_ERROR("Could not set model state");
        }
    }

    void CModel::SetParent(CModel* lp_parent, dfv::Vector3 parent_joint_position)
    {
        this->lp_parent = lp_parent;
        this->parent_joint_position = parent_joint_position;
    }

    dfv::Vector3 CModel::GetPosition()
    {
        if (this->lp_parent != NULL)
        {   
            return this->lp_parent->GetPosition() + this->parent_joint_position.GetRotated(this->lp_parent->GetOrientation());
        }
        else
        {
            return this->position;
        }
    }

    dfv::Quaternion CModel::GetOrientation()
    {
        return this->orientation;
    }


    void CModel::SetId(int id)
    {
        this->id = id;
    }

    int CModel::GetId()
    {
        return this->id;
    }

}
