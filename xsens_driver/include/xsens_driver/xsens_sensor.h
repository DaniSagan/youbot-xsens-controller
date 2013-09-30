/*
 * Clase Sensor en la que la clase Driver almacenará 
 * los datos obtenidos por los sensores físicos Xsens, 
 * además de ciertos parámetros de configuración
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#ifndef XSENS_SENSOR_H
#define XSENS_SENSOR_H

#include <xsens_driver/cmtdef.h>

namespace xsens
{
    class Sensor
    {
        public:
            Sensor();
            ~Sensor();
            
            void SetAlignmentMatrix(const CmtMatrix& matrix);
            
            CmtMatrix           alignment_matrix;
            
            CmtOutputMode       GetOutputMode() const;
            void                SetOutputMode(const CmtOutputMode output_mode);
            CmtOutputSettings   GetOutputSettings() const;
            void                SetOutputSettings(const CmtOutputSettings output_settings);
            
        protected:
        private:
            CmtCalData          calibrated_data;
            CmtQuat             quaternion_data;
            CmtEuler            euler_data;
            CmtMatrix           matrix_data;
            CmtRawData          raw_data;
            CmtVector           position_lla;
            CmtGpsPvtData       gps_pvt_data;
            double              temperature_data;
            
            CmtDeviceId         device_id;
            
            CmtOutputMode       output_mode;
            CmtOutputSettings   output_settings;
            
            friend class Driver;
            
    };
}

#endif
