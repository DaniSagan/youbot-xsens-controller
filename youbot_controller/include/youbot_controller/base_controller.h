#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <vector>
#include <dfv/dfv.h>
#include "youbot_controller/controller.h"

class BaseController: public Controller
{
public:
    BaseController(dfv::XsensListener& sensors_, dfv::Youbot& youbot_);
    ~BaseController();
    
    std::vector<float> GetVels();
    virtual std::vector<float> OnUpdate();

protected:
    
};

#endif
