#include "youbot_controller/arm_controller.h"

PlatformController::PlatformController()
{
    this->state.resize(3);
}

PlatformController::~PlatformController()
{
}
    
void PlatformController::Update(dfv::Vector3 acc)
{
}

std::vector<double> PlatformController::GetState()
{
    return this->state;
}
