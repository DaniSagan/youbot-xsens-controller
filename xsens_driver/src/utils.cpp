#include <xsens_driver/utils.h>

namespace xsens
{
    CmtMatrix DfvToCmtMatrix(const dfv::Matrix& m)
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
    
    void operator<<(geometry_msgs::Vector3& msg, const dfv::Vector3& v)
    {
        msg.x = v.x;
        msg.y = v.y;
        msg.z = v.z;
    }
    
    void operator<<(geometry_msgs::Vector3Stamped& msg, const dfv::Vector3& v)
    {
        msg.header.stamp = ros::Time::now();
        msg.vector.x = v.x;
        msg.vector.y = v.y;
        msg.vector.z = v.z;
    }
}
