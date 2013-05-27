#include <xsens_driver/utils.h>

namespace xsens
{
    const CmtMatrix DfvToCmtMatrix(const dfv::Matrix& m)
    {
        CmtMatrix result;
        if(m.GetRows() == 3 && m.GetColumns() == 3)
        {
            for(unsigned int j = 0; j < m.GetRows(); ++j)
            {
                for(unsigned int i = 0; i < m.GetColumns(); ++i)
                {
                    result.m_data[j][i] = m.Get(j, i);
                }
            }
        }
        return result;
    }
    
    const dfv::Matrix CmtToDfvMatrix(const CmtMatrix& m)
    {
        dfv::Matrix result(3, 3);
        for(unsigned int j = 0; j < 3; ++j)
        {
            for(unsigned int i = 0; i < 3; ++i)
            {
                result.Set(j, i, m.m_data[j][i]);
            }
        }
        return result;
    }
    
    const CmtVector DfvToCmtVector(const dfv::Vector3& v)
    {
        CmtVector result;
        result.m_data[0] = v.x;
        result.m_data[1] = v.y;
        result.m_data[2] = v.z;
        return result;
    }
    
    const dfv::Vector3 CmtToDfvShortVector(const CmtShortVector& v)
    {
        return dfv::Vector3(v.m_data[0], v.m_data[1], v.m_data[2]);
    }
    
    const CmtShortVector DfvToCmtShortVector(const dfv::Vector3& v)
    {
        CmtShortVector result;
        result.m_data[0] = v.x;
        result.m_data[1] = v.y;
        result.m_data[2] = v.z;
        return result;
    }
    
    const dfv::Vector3 CmtToDfvVector(const CmtVector& v)
    {
        return dfv::Vector3(v.m_data[0], v.m_data[1], v.m_data[2]);
    }
    
    const geometry_msgs::Vector3 ToVector3Msg(const dfv::Vector3& v)
    {
        geometry_msgs::Vector3 msg;
        msg.x = v.x;
        msg.y = v.y;
        msg.z = v.z;
        return msg;
    }
    
    const geometry_msgs::Vector3Stamped ToVector3StampedMsg(const dfv::Vector3& v)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.vector.x = v.x;
        msg.vector.y = v.y;
        msg.vector.z = v.z;
        return msg;
    }
}
