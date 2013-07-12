#include <xsens_driver/xsens_sensor_subscriber.h>

int main(int argc, char** argv)
{
    // Inicialización de ROS
    ros::init(argc, argv, "xsens_sensr_subscriber_node");
    ros::NodeHandle node_handle;
    
    // Declaracion del objeto sensor_subscriber_list
    // Se autoconfigura de acuerdo a los parámetros detectados
    // e inmediatamente comienza la lectuira de los datos
    xsens::SensorSubscriberList sensor_subscriber_list(node_handle);
    
    while(ros::ok())
    {
        // Código para lectura y procesamiento de datos de sensor_subscriber_list
        // ...
        
        // Bucle a 10 Hz
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}
