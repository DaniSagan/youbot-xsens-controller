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
            
        protected:
        private:
            CmtCalData          calibrated_data;
            CmtQuat             quaternion_data;
            CmtEuler            euler_data;
            CmtMatrix           matrix_data;
            CmtRawData          raw_data;
            double              temperature_data;
            
            CmtDeviceId         device_id;
            
            friend class Driver;
            
    };
}

#endif
