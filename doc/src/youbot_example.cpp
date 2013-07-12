#include <youbot_controller/youbot.h>

int main(int argc, char** argv)
{
    // inicialización de ROS
    ros::init(argc, argv, "youbot_example");
    ros::NodeHandle node_handle;
    
    // Declaración de un objeto Youbot
    Youbot youbot(node_handle);
    
    while(ros::ok())
    {
        // Asignación de los valores objetivo de cada articulación
        youbot.joint_positions[0] = 3.141592;
        youbot.joint_positions[1] = 2.718182;
        youbot.joint_positions[2] = -1.618033;
        youbot.joint_positions[3] = 1.414213;
        youbot.joint_positions[4] = 0.577215;
        
        // Publicación del mensaje
        youbot.PublishMessage();
        
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    return 0;
}
