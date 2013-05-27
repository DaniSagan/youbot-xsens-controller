#ifndef XSENS_DRIVER_UTILS_H
#define XSENS_DRIVER_UTILS_H

#include <xsens_driver/cmtdef.h>
#include <dfv/dfv.h>

namespace xsens
{
    const CmtMatrix       DfvToCmtMatrix(const dfv::Matrix& m);
    const dfv::Matrix     CmtToDfvMatrix(const CmtMatrix& m);
    const CmtVector       DfvToCmtVector(const dfv::Vector3& v);
    const dfv::Vector3    CmtToDfvVector(const CmtVector& v);
    const CmtShortVector  DfvToCmtShortVector(const dfv::Vector3& v);
    const dfv::Vector3    CmtToDfvShortVector(const CmtShortVector& v);
    
    
    const geometry_msgs::Vector3 ToVector3Msg(const dfv::Vector3& v);
    const geometry_msgs::Vector3Stamped ToVector3StampedMsg(const dfv::Vector3& v);    
}

#endif
