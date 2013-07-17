#include <iostream>

#include "gazebo_model_list.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_node");
    ros::NodeHandle node_handle;
    
    gazebo::CModelList model_list(node_handle);
    model_list.AddModel("model1", CQuaternion(0.0,0.0,0.0,0.0), "model/box.xml");
    model_list.AddModel("model2", CQuaternion(0.0,0.0,2.0,0.0), "model/box.xml");
    //model_list.AddModel("model3", CQuaternion(0.0,0.0,4.0,0.0), "model/box.xml");
    model_list.SubscribeModelToTopic(0, "xsens/0/ori_quat");
    //model_list.SubscribeModelToTopic(0, "xsens/own_quat");
    model_list.SubscribeModelToTopic(1, "xsens/0/calibrated_quat");
    //model_list.SubscribeModelToTopic(2, "xsens/2/ori_quat");
    //model_list.SetJoint(0, 1, CQuaternion(0.0, 0.0, 2.0, 0.0));
    //model_list.SetJoint(1, 2, CQuaternion(0.0, 0.0, 2.0, 0.0));
    model_list.EnableGazebo(true);
    //model_list.EnableRviz(true);
    model_list.Spawn();
    
    ros::spin();
    
    return 0;
}

