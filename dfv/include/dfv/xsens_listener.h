/*
 * Clases SensorSubscriber y XsensListener
 * 
 * Estas clases no son utilizadas por el driver. Forman parte de la librería 
 * dfv. Proporcionan una interfaz sencilla para acceder a los datos publicados 
 * en ROS por el driver que se puede utilizar en otros programas.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo 2013
 *
 */

#ifndef XSENS_LISTENER_H
#define XSENS_LISTENER_H

#include <dfv/dfv.h>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

//! El espacio de nombres para la librería dfv
namespace dfv
{
    /*! \brief Encapsula las comunicaciones de ROS de un sensor Xsens.
    
        No es necesario utilizar esta clase ya que de su funcionamiento se encarga
        la clase XsensListener.
    */
    class SensorSubscriber
    {
        public:
            /*! Constructor por defecto
            */
            SensorSubscriber(unsigned int mt_index_, ros::NodeHandle& node_handle_);
            
            /*! Destructor
            */
            ~SensorSubscriber();
            
            /*! Realiza la subscripción a los topics publicados por el programa
                xsens_driver.
            */
            bool                    SubscribeToTopics();
            
            /*! Devuelve los valores leídos de los acelerómetros
                
                Función que devuelve el vector con las componentes en
                los ejes x, y, z en \f$\frac{m}{s^2}\f$ leídas de los acelerómetros
            */
            const dfv::Vector3      GetAcc() const;
            
            /*! Devuelve los valores leídos de los giróscopos
                
                Función que devuelve el vector con las velocidades angulares en
                los ejes x, y, z en \f$\frac{rad}{s}\f$ leídas de los acelerómetros
            */
            const dfv::Vector3      GetGyr() const;
            
            /*! Devuelve los valores leídos de los magnetómetros
                
                Función que devuelve el vector con las componentes en
                los ejes x, y, z en unidades arbitrarias (normalizadas según la 
                intensidad del campo magnético registrado) leídas de los magnetómetros
            */
            const dfv::Vector3      GetMag() const;
            
            /*! Devuelve el cuaternión de orientación calculado por el propio
                sensor.
            */
            const dfv::Quaternion   GetOriQuat() const;
            
            /*! Devuelve la matriz de orientación calculada por el propio
                sensor.
            */
            const dfv::Matrix       GetOriMatrix() const;
            
            /*! Devuelve los ángulos de Euler calculados por el propio sensor.
            */
            const dfv::Vector3      GetOriEuler() const;
            
            
            
        private:
            ros::NodeHandle&    node_handle;
            unsigned int        mt_index;
            
            std::string         acc_topic_name;
            std::string         gyr_topic_name;
            std::string         mag_topic_name;
            std::string         ori_quat_topic_name;
            std::string         ori_matrix_topic_name;
            std::string         ori_euler_topic_name;
            
            dfv::Vector3        acc;
            dfv::Vector3        gyr;
            dfv::Vector3        mag;
            
            dfv::Quaternion     ori_quat;
            dfv::Matrix         ori_matrix;
            dfv::Vector3        ori_euler;
            
            dfv::Vector3        position_lla;            
            double              temperature;
            
            ros::Subscriber     acc_subscriber;
            ros::Subscriber     gyr_subscriber;
            ros::Subscriber     mag_subscriber;
            ros::Subscriber     ori_quat_subscriber;
            ros::Subscriber     ori_matrix_subscriber;
            ros::Subscriber     ori_euler_subscriber;
            
            void                AccSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                GyrSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                MagSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
            void                OriQuatSubCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
            void                OriMatrixSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void                OriEulerSubCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    };
    
    
    /*! \brief Encapsula las comunicaciones de ROS de una red de sensores Xsens.
    
        Esta clase se encarga de declarar y controlar el funcionamiento de la
        clase SensorSubscriber.
    */
    class XsensListener
    {
        public:
            /*! \brief Constructor por defecto.
                
                \param node_handle_ Handle del nodo donde lo declaremos.
            */
            XsensListener(ros::NodeHandle& node_handle_);
            
            /*! \brief Destructor
            */
            ~XsensListener(); 

            //! Función que devuelve el número de sensores detectados
            unsigned int GetMtCount() const;
            
            
            /*! \brief Devuelve los valores leídos de los acelerómetros del sensor
                especificado.
                
                Función que devuelve el vector con las componentes en
                los ejes x, y, z en \f$\mbox{m/s}^2\f$ leídas de los acelerómetros
                \param mt_index ID del sensor
                \return vector con las componentes en los ejes x, y, z en 
                \f$\mbox{m/s}^2\f$ leídas de los acelerómetros
            */
            const dfv::Vector3      GetAcc(unsigned int mt_index = 0) const;
            
            /*! \brief Devuelve los valores leídos de los giróscopos del sensor
                especificado.
                
                \param mt_index id del sensor
                \return Vector de 3 elementos con las velocidades angulares en
                los ejes x, y, z en \f$\mbox{rad/s}\f$ leídas de los giróscopos
            */
            const dfv::Vector3      GetGyr(unsigned int mt_index = 0) const;
            
            /*! \brief Devuelve los valores leídos de los magnetómetros del
                sensor especificado

                \param mt_index ID del sensor
                \return Vector de 3 elementos con las componentes en
                los ejes x, y, z en unidades arbitrarias (normalizadas según la 
                intensidad del campo magnético registrado) leídas de los magnetómetros
            */
            const dfv::Vector3      GetMag(unsigned int mt_index = 0) const;
            
            /*! \brief Devuelve el cuaternión de orientación calculado por el propio sensor.
            
                Este valor es válido sólo si se ha configurado el driver xsens_node
                para que publique cuaterniones de orientación.
        
                \param mt_index ID del sensor
                \return Cuaternión de orientación
            */
            const dfv::Quaternion   GetOriQuat(unsigned int mt_index = 0) const;
            
            /*! \brief Devuelve la matriz de orientación calculada por el propio sensor.
            
                Este valor es válido sólo si se ha configurado el driver xsens_node
                para que publique matrices de orientación.
                
                \param mt_index ID del sensor
                \return Matriz 3x3 de orientación
            */
            const dfv::Matrix       GetOriMatrix(unsigned int mt_index = 0) const;
            
            /*! \brief Devuelve un vector con los ángulos de Euler calculados por el propio sensor.
                
                Este valor es válido sólo si se ha configurado el driver xsens_node
                para que publique ángulos de Euler.
                
                \param mt_index ID del sensor
                \return Vector con los ángulos en los ejes x, y, z respectivamente
            */
            const dfv::Vector3      GetOriEuler(unsigned int mt_index = 0) const;
            
        private:            
            ros::NodeHandle& node_handle;
            unsigned int mt_count;
            std::vector<SensorSubscriber*> sensors;
            
    };
}

#endif
