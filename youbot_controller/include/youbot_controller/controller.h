#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>

class Controller
{
public:
    Controller();
    ~Controller();
    
    virtual void Update(std::vector<dfv::Quaternion> links_ori);
    std::vector<double> GetState() const;
    unsigned int GetTicks() const;
    
protected:    
    std::vector<double> state;
    unsigned int ticks;
};


#endif
