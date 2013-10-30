#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <dfv/dfv.h>

class Controller
{
public:
    Controller(dfv::XsensListener& sensors_, dfv::Youbot& youbot_);
    ~Controller();
    
    virtual bool OnInit();
    virtual std::vector<float> OnUpdate();
    std::vector<float> GetState() const;
    unsigned int GetFrame() const;
    
protected:    
    std::vector<float> state;
    unsigned int frame;
    dfv::XsensListener& sensors;
    dfv::Youbot& youbot;
};


#endif
