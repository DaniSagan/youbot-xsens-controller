#include <ros/ros.h>
#include <dfv/dfv.h>
#include <xsens_driver/xsens_sensor_subscriber.h>
#include <arm_visualizer/gazebo_model_list.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_visualizer");
    ros::NodeHandle node_handle;
    
    xsens::SensorSubscriberList sensors(node_handle);
    
    gazebo::CModelList arm(node_handle);
    
    double arm_length = 0.30;
    double forearm_length = 0.25;
    double hand_length = 0.17;
    
    arm.AddModel("arm", dfv::Vector3(0.0,0.0,0.0), "model/arm.xml");
    arm.AddModel("forearm", dfv::Vector3(0.0,0.0,0.0), "model/forearm.xml");
    arm.AddModel("hand", dfv::Vector3(0.0,0.0,0.0), "model/hand.xml");
    arm.Spawn();
    
    arm.SetJoint(0, 1, dfv::Vector3(arm_length, 0.0, 0.0));
    arm.SetJoint(1, 2, dfv::Vector3(forearm_length, 0.0, 0.0));
    
    while(ros::ok())
    {
        for(unsigned int i = 0; i < arm.GetCount(); i++)
        {
            arm.SetOrientation(i, sensors.GetOriQuat(i));
        }
        
        /* Código de prueba con un sensor
        arm.SetOrientation(0, sensors.GetOriQuat(0));
        arm.SetOrientation(1, dfv::Quaternion::identity);
        arm.SetOrientation(2, sensors.GetOriQuat(0).GetConjugate());
        std::cout << "ORI: " << sensors.GetOriQuat(0) << std::endl;
        */
        arm.PublishMessage();
        ros::spinOnce();
        //ros::Duration(0.1).sleep();
    }
    
    return 0;
}
