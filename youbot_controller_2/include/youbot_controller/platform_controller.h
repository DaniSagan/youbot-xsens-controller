#ifndef PLATFORM_CONTROLLER_H
#define PLATFORM_CONTROLLER_H

#include <dfv/dfv.h>

class PlatformController
{
public:
    PlatformController();
    ~PlatformController();
    
    void Update(dfv::Vector3 acc);
    std::vector<double> GetState();

private:
    std::vector<double> state;
};

#endif
