#include <arm_visualizer/gazebo_model_list.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_visualizer");
    ros::NodeHandle node_handle;
    
    // Decalaración del objeto CModelList
    // para crear el modelo del brazo
    gazebo::CModelList arm(node_handle);
    
    double arm_length = 0.30; // longitud del brazo
    double forearm_length = 0.25; // longitud del antebrazo
    double hand_length = 0.17; // longitud de la mano
    
    // Creación de los eslabones para el brazo, antebrazo y mano
    arm.AddModel("arm", dfv::Vector3(0.0,0.0,0.0), "model/arm.xml");
    arm.AddModel("forearm", dfv::Vector3(0.0,0.0,0.0), "model/forearm.xml");
    arm.AddModel("hand", dfv::Vector3(0.0,0.0,0.0), "model/hand.xml");
    
    // Creación de las articulaciones para codo y muñeca
    arm.SetJoint(0, 1, dfv::Vector3(arm_length, 0.0, 0.0));
    arm.SetJoint(1, 2, dfv::Vector3(forearm_length, 0.0, 0.0));
    
    // Inserción del brazo en la simulación
    arm.Spawn();
    
    while(ros::ok())
    {
        
        /* 
        Asignamos las orientaciones de los eslabones
        arm.SetOrientation(0, ...);
        arm.SetOrientation(1, ...);
        arm.SetOrientation(2, ...);
        */
        
        // Publicamos el mensaje con el estado de los eslabones
        arm.PublishMessage();
        ros::spinOnce();
    }
    
    return 0;
}
