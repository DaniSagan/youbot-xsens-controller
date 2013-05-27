#ifndef XSENS_DRIVER_UTILS_H
#define XSENS_DRIVER_UTILS_H

#include <xsens_driver/cmtdef.h>
#include <dfv/dfv.h>

namespace xsens
{
    CmtMatrix DfvToCmtMatrix(const dfv::Matrix& m);
    void operator<<(geometry_msgs::Vector3& msg, const dfv::Vector3& v);
    void operator<<(geometry_msgs::Vector3Stamped& msg, const dfv::Vector3& v);    
}

#endif
