#include <xsens_driver/xsens_sensor.h>

namespace xsens
{
    Sensor::Sensor()
    {
        CmtMatrix matrix;
        matrix.m_data[0][0] = 1.0; matrix.m_data[0][1] = 0.0; matrix.m_data[0][2] = 0.0;
        matrix.m_data[1][0] = 0.0; matrix.m_data[1][1] = 1.0; matrix.m_data[1][2] = 0.0;
        matrix.m_data[2][0] = 0.0; matrix.m_data[2][1] = 0.0; matrix.m_data[2][2] = 1.0;
        this->alignment_matrix = matrix;
    }
    
    Sensor::~Sensor()
    {
    }
    
    void Sensor::SetAlignmentMatrix(const CmtMatrix& matrix)
    {
        this->alignment_matrix = matrix;
    }
}
