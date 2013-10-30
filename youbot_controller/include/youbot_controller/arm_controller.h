#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <vector>
#include <dfv/dfv.h>
#include "youbot_controller/controller.h"

class ArmController: public Controller
{
public:
    ArmController(dfv::XsensListener& sensors_, dfv::Youbot& youbot_);
    ~ArmController();
    
    bool OnInit();
    std::vector<float> OnUpdate();
    std::vector<float> GetAngles();
    
protected:
    float ang_0_offset;
};

#endif
