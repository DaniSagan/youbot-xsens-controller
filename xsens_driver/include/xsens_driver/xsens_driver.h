/*
 * Clase Driver encargada de la configuración
 * y toma de datos de los sensores Xsens
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#ifndef XSENS_DRIVER_H
#define XSENS_DRIVER_H

#include <vector>
#include <sstream>

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <ros/ros.h>

#include <xsens_driver/cmtdef.h>
#include <xsens_driver/xsens_time.h>
#include <xsens_driver/xsens_list.h>
#include <xsens_driver/cmtscan.h>
#include <xsens_driver/cmt3.h>
#include <xsens_driver/xsens_sensor.h>

namespace xsens
{
    
    class Driver
    {
        public:
            Driver();
            ~Driver();
            
            bool Initialize();
            
            void SetOutputMode(CmtOutputMode output_mode);
            CmtOutputMode GetOutputMode() const;
            
            void SetOutputSettings(CmtOutputSettings output_settings);
            CmtOutputSettings GetOutputSettings() const;
            
            void SetAlignmentMatrix(unsigned int sensor_index, CmtMatrix alignment_matrix);
            
            bool SpinOnce();
            bool RetrieveData();
            unsigned int  GetMtCount();
            CmtOutputMode GetOutputMode();
            CmtOutputSettings GetOutputSettings();
            
            // functions for getting data
            CmtQuat&    GetOriQuat(int mt_index = 0);
            CmtMatrix&  GetOriMatrix(int mt_index = 0);
            CmtEuler&   GetOriEuler(int mt_index = 0);
            CmtRawData& GetRawData(int mt_index = 0);
            CmtCalData& GetCalData(int mt_index = 0);
            CmtVector&  GetPositionLLA(int mt_index = 0);
            
            // Vector de sensores
            std::vector<Sensor> v_sensors;
         
        private:
            Cmt3                cmt3;
            unsigned int        mt_count;
            CmtOutputMode       output_mode;
            CmtOutputSettings   output_settings;
            short               skip_factor;
            short               skip_factor_count;
            
            Packet* lp_packet;
            
            unsigned short          sample_data;
            
            bool DoHardwareScan();
            bool SetConfiguration();
                    
    };
};

#endif
