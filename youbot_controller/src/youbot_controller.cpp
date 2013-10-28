/*
 * Programa que toma los datos de los topics
 * de tres sensores xsens, calcula los ángulos
 * de rotación entre cada sensor y los publica 
 * en el topic para mover el brazo robótico del
 * robot Youbot.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>

std::vector<float> GetArmAngles(dfv::XsensListener& sensors);
std::vector<float> GetBaseVel(dfv::XsensListener& sensors);

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "youbot_controller");
    ros::NodeHandle node_handle;    
    dfv::XsensListener sensors(node_handle);
    
    // Si no hay sensores, salimos del programa
    if(sensors.Count() == 0)
    {
        ROS_ERROR("No MTs found. Quitting...");
        return 1;
    }
    ROS_INFO("Detected IMU count: %d", sensors.Count());
    if(sensors.Count() != 4)
    {
        ROS_WARN("4 sensors are needed for this demo to work properly. " 
            "Please connect 4 Xsens IMUs to the Xbus Master and restart the program. "
            "Quitting...");
        return 1;
    }
    
    // Esperamos a que ROS llame a las funciones callback para
    // leer los datos de los topics
    ROS_INFO("Waiting for the topics to be read...");
    while(sensors.GetOriQuat(0) == dfv::Quaternion(0.0, 0.0, 0.0, 0.0))
    {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Creating youbot...");
    dfv::Youbot youbot(node_handle);
    youbot.arm.Enable();
    youbot.base.Disable();
    
    ROS_INFO("Youbot Created");
    std::vector<float> angs;
    std::vector<float> vels;
    
    unsigned int c = 50;
    double ang_0_offset = 0;
    ROS_INFO("Calibrating Sensor Orientation...");
    
    while(ros::ok() && c != 0)
    {
        angs = GetArmAngles(sensors);
        ang_0_offset = angs[0];
        ros::Duration(0.05).sleep();
        c--;
    }
    ROS_INFO("Calibration Done.");
    
    // bucle principal
    while(ros::ok())
    {
        // Obtención de los ángulos a partir de los cuaterniones de los sensores
        angs = GetArmAngles(sensors);        
        angs[0] = dfv::NormalizeAngle(angs[0] + ang_0_offset);        
        youbot.arm.SetPos(angs);
        
        // Obtención de los ángulos del cuarto sensor
        vels = GetBaseVel(sensors);
        youbot.base.Move(vels[0], vels[1], vels[2]);
        
        youbot.Wait(0.05f);
        ros::spinOnce();
    }
    
    return 0;
}

std::vector<float> GetArmAngles(dfv::XsensListener& sensors)
{    
    dfv::Quaternion q0 = sensors.GetOriQuat(0);
    dfv::Quaternion q1 = sensors.GetOriQuat(1);
    dfv::Quaternion q2 = sensors.GetOriQuat(2);

    dfv::Quaternion q01 = q0.GetConjugate() * q1;
    dfv::Quaternion q12 = q1.GetConjugate() * q2;
    
    // Cálculo de los ángulos entre cada sensor
    
    float yaw_0;
    float pitch_0;
    float pitch_1;
    float pitch_2;
    
    // ejes locales
    dfv::Vector3 ii = dfv::Vector3::i.GetRotated(q0);
    dfv::Vector3 jj = dfv::Vector3::j.GetRotated(q0);
    dfv::Vector3 kk = dfv::Vector3::k.GetRotated(q0);
    
    if (kk.z >= 0)
    {
        //yaw_0 = atan2(ii.y, ii.x);
        yaw_0 = atan2(-jj.x, jj.y);
        pitch_0 = atan2(ii.z, sqrt(ii.x*ii.x + ii.y*ii.y));
    }
    else
    {
        yaw_0 = atan2(-jj.x, jj.y);
        pitch_0 = atan2(ii.z, -sqrt(ii.x*ii.x + ii.y*ii.y));
    }
    
    ii = dfv::Vector3::i.GetRotated(q01);
    jj = dfv::Vector3::j.GetRotated(q01);
    kk = dfv::Vector3::k.GetRotated(q01);
    
    pitch_1 = atan2(ii.z, ii.x);
    
    ii = dfv::Vector3::i.GetRotated(q12);
    jj = dfv::Vector3::j.GetRotated(q12);
    kk = dfv::Vector3::k.GetRotated(q12);
    
    pitch_2 = atan2(ii.z, ii.x);
    
    // Adaptación de los ángulos obtenidos
    // teniendo en cuenta los offsets de cada
    // articulación
    
    //std::cout << "pitch_0: " << pitch_0 << std::endl;
    //std::cout << "pitch_1: " << pitch_1 << std::endl;
    //std::cout << "pitch_2: " << pitch_2 << std::endl;
    
    //double offsets[] = {2.9597f, -0.4263f, -2.5712f, 1.8111f, 2.9124f};
    
    //double angs[5];
    /*angles[0] = offsets[0] - yaw_0;
    angles[1] = offsets[1] + pitch_0;
    angles[2] = offsets[2] + pitch_1;
    angles[3] = offsets[3] + pitch_2;
    angles[4] = offsets[4];*/
    
    std::vector<float> angles(5);
    angles[0] = -yaw_0;
    angles[1] = pitch_0 - dfv::pi/2.0;
    angles[2] = pitch_1;
    angles[3] = pitch_2;
    angles[4] = 0.f;
    
    return angles;
}

std::vector<float> GetBaseVel(dfv::XsensListener& sensors)
{
    dfv::Vector3 acc_4 = sensors.GetAcc(3);        
    float ang_turn = atan2(acc_4.y, acc_4.x);
    float ang_acc = atan2(acc_4.z, acc_4.x);
    float ang_vel = 0.0;
    float lin_vel = 0.0;
    
    if (ang_acc < 0.4 * dfv::pi && ang_acc > -0.4 * dfv::pi &&
       ang_turn < 0.4 * dfv::pi && ang_turn > -0.4 * dfv::pi)
    {
        if (ang_turn >= 0.1)
        {
            ang_vel = (ang_turn - 0.1) * 2.0;
        }
        else if (ang_turn <= -0.1)
        {
            ang_vel = (ang_turn + 0.1) * 2.0;
        }
        else
        {
            ang_vel = 0.0;
        }
        ang_vel = ang_vel * (-1.0);
        
        if (ang_acc >= 0.1)
        {
            lin_vel = (ang_acc - 0.1) * 0.5;
            //ang_vel = ang_vel * (-1.0);
        }
        else if (ang_acc <= -0.1)
        {
            lin_vel = (ang_acc + 0.1) * 0.5;
            
        }
        else
        {
            lin_vel = 0.0;
        }
        
    }    
    else
    {
        ang_vel = 0.0;
        lin_vel = 0.0;
    }
    
    std::vector<float> vels(3);
    vels[0] = lin_vel;
    vels[1] = 0.f;
    vels[2] = ang_vel;
    return vels;
    
    //youbot.linear_vel = dfv::Vector3(lin_vel, 0.0, 0.0);
    //youbot.angular_vel = dfv::Vector3(0.0, 0.0, ang_vel);
}
