#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <vector>
#include <dfv/dfv.h>

class ArmController
{
public:
    ArmController();
    ~ArmController();
    
    void Update(dfv::Quaternion arm_ori,
                dfv::Quaternion forearm_ori,
                dfv::Quaternion hand_ori);
    std::vector<double> GetState();
    
private:
    std::vector<double> state;
};

#endif
