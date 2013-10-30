#include "youbot_controller/controller.h"

Controller::Controller(dfv::XsensListener& sensors_, dfv::Youbot& youbot_):
    sensors(sensors_),
    youbot(youbot_)
{
}

Controller::~Controller()
{
}

bool Controller::OnInit()
{
    return true;
}

std::vector<float> Controller::OnUpdate()
{
    this->frame++;
    return this->state;
}

std::vector<float> Controller::GetState() const
{
    return this->state;
}

unsigned int Controller::GetFrame() const
{
    return this->frame;
}
