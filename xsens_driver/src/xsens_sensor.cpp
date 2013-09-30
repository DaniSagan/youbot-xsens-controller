#include <xsens_driver/xsens_sensor.h>

namespace xsens
{
    Sensor::Sensor():
        output_mode(CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT),
        output_settings(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT)
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
    
    CmtOutputMode Sensor::GetOutputMode() const
    {
        return this->output_mode;
    }
    
    void Sensor::SetOutputMode(const CmtOutputMode output_mode)
    {
        this->output_mode = output_mode;
    }
    
    CmtOutputSettings Sensor::GetOutputSettings() const
    {
        return this->output_settings;
    }
    
    void Sensor::SetOutputSettings(const CmtOutputSettings output_settings)
    {
        this->output_settings = output_settings | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    }
}
