#include "youbot_controller/controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}

void Update(std::vector<dfv::Quaternion> links_ori)
{
    this->ticks++;
}

std::vector<double> GetState() const;
{
    return this->state;
}

unsigned int GetTicks() const
{
    return this->ticks;
}
