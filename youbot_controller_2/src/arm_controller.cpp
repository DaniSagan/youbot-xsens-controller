#include "youbot_controller/arm_controller.h"

ArmController::ArmController()
{
    this->state.resize(5);
}

ArmController::~ArmController()
{
}
    
void ArmController::Update(dfv::Quaternion arm_ori,
                           dfv::Quaternion forearm_ori,
                           dfv::Quaternion hand_ori)
{
    
    //dfv::Quaternion q0 = sensors.GetOriQuat(0);
    //dfv::Quaternion q1 = sensors.GetOriQuat(1);
    //dfv::Quaternion q2 = sensors.GetOriQuat(2);
    
    dfv::Quaternion shoulder_quat = arm_ori;
    dfv::Quaternion elbow_quat = arm_ori.GetConjugate() * forearm_ori;
    dfv::Quaternion wrist_quat = forearm_ori.GetConjugate() * jand_ori;
    //dfv::Quaternion q01 = q0.GetConjugate() * q1;
    //dfv::Quaternion q12 = q1.GetConjugate() * q2;
    
    // Cálculo de los ángulos entre cada sensor
    
    double yaw_0;
    double pitch_0;
    double pitch_1;
    double pitch_2;
    
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
    
    std::cout << "pitch_0: " << pitch_0 << std::endl;
    std::cout << "pitch_1: " << pitch_1 << std::endl;
    std::cout << "pitch_2: " << pitch_2 << std::endl;
    
    double offsets[] = {2.9597f, -0.4263f, -2.5712f, 1.8111f, 2.9124f};
    
    double angs[5];
    angles[0] = offsets[0] - yaw_0;
    angles[1] = offsets[1] + pitch_0;
    angles[2] = offsets[2] + pitch_1;
    angles[3] = offsets[3] + pitch_2;
    angles[4] = offsets[4];
    
    return angles;
}

std::vector<double> ArmController::GetState()
{
    return this->state;
}
