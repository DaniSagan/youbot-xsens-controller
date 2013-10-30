#include "youbot_controller/base_controller.h"

BaseController::BaseController(dfv::XsensListener& sensors_, dfv::Youbot& youbot_):
    Controller(sensors_, youbot_)
{
    this->state.resize(3);
}

BaseController::~BaseController()
{
}
    
std::vector<float> BaseController::OnUpdate()
{   
    this->state = this->GetVels();
    youbot.base.Move(this->state[0], this->state[1], this->state[2]);    
    this->frame++;
    return this->state;
}

std::vector<float> BaseController::GetVels()
{
    dfv::Vector3 acc_4 = this->sensors.GetAcc(3);        
    float ang_turn = atan2(acc_4.y, acc_4.x);
    float ang_acc = atan2(acc_4.z, acc_4.x);
    float ang_vel = 0.0;
    float lin_vel = 0.0;
    
    if (ang_acc  < 0.4 * dfv::pi && ang_acc  > -0.4 * dfv::pi &&
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
}
