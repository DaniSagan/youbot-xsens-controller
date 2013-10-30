#include "youbot_controller/arm_controller.h"

ArmController::ArmController(dfv::XsensListener& sensors_, dfv::Youbot& youbot_):
    Controller(sensors_, youbot_)
{
    this->state.resize(5);
}

ArmController::~ArmController()
{
}
 
bool ArmController::OnInit()
{
    unsigned int n = 0;
    this->ang_0_offset = 0.f;
    
    while(n < 50)
    {
        n++;
        std::vector<float> angs = this->GetAngles();
        this->ang_0_offset = ((float(n)-1.0)*this->ang_0_offset + angs[0])/float(n);
        ros::Duration(0.05).sleep();        
    }
    return true;
} 

std::vector<float> ArmController::OnUpdate()
{
    this->state = GetAngles();
    this->state[0] = dfv::NormalizeAngle(this->state[0] + this->ang_0_offset);
    youbot.arm.SetPos(this->state);
    this->frame++;
    return this->state;    
}

    
std::vector<float> ArmController::GetAngles()
{
    // cuaterniones de brazo, antebrazo y mano
    dfv::Quaternion q0 = this->sensors.GetOriQuat(0);
    dfv::Quaternion q1 = this->sensors.GetOriQuat(1);
    dfv::Quaternion q2 = this->sensors.GetOriQuat(2);
    
    // cuaterniones locales de brazo-antebrazo y antebrazo-muñeca
    dfv::Quaternion q01 = q0.GetConjugate() * q1;
    dfv::Quaternion q12 = q1.GetConjugate() * q2;
    
    // Cálculo de los ángulos entre cada sensor
    
    float yaw_0;
    float pitch_0;
    float pitch_1;
    float pitch_2;
    
    // ejes locales brazo
    dfv::Vector3 ii = dfv::Vector3::i.GetRotated(q0);
    dfv::Vector3 jj = dfv::Vector3::j.GetRotated(q0);
    dfv::Vector3 kk = dfv::Vector3::k.GetRotated(q0);
    
    // ángulo en eje Y y Z cuerpo-brazo
    if (kk.z >= 0)
    {
        yaw_0 = atan2(-jj.x, jj.y);
        pitch_0 = atan2(ii.z, sqrt(ii.x*ii.x + ii.y*ii.y));
    }
    else
    {
        yaw_0 = atan2(-jj.x, jj.y);
        pitch_0 = atan2(ii.z, -sqrt(ii.x*ii.x + ii.y*ii.y));
    }
    
    // ejes locales antebrazo
    ii = dfv::Vector3::i.GetRotated(q01);
    jj = dfv::Vector3::j.GetRotated(q01);
    kk = dfv::Vector3::k.GetRotated(q01);
    
    // ángulo en eje Y brazo-antebrazo
    pitch_1 = atan2(ii.z, ii.x);
    
    // ejes locales mano
    ii = dfv::Vector3::i.GetRotated(q12);
    jj = dfv::Vector3::j.GetRotated(q12);
    kk = dfv::Vector3::k.GetRotated(q12);
    
    // ángulo en eje Y antebrazo-mano
    pitch_2 = atan2(ii.z, ii.x);
    
    std::vector<float> angs(5);
    angs[0] = -yaw_0;
    angs[1] = pitch_0 - dfv::pi/2.0;
    angs[2] = pitch_1;
    angs[3] = pitch_2;
    angs[4] = 0.f;    
    
    return angs;
}

