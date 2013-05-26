#include <xsens_driver/xsens_driver.h>

namespace xsens
{

    Driver::Driver():
        mt_count(0),
        output_mode(CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT),
        output_settings(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION),
        skip_factor(10),
        skip_factor_count(0),
        lp_packet(NULL)
    {
    }

    Driver::~Driver()
    {
        delete this->lp_packet;
        this->cmt3.closePort();
    }
    
    void Driver::SetOutputMode(CmtOutputMode output_mode)
    {
        this->output_mode = output_mode;
    }
    
    void Driver::SetOutputSettings(CmtOutputSettings output_settings)
    {
        this->output_settings = output_settings;
    }
    
    void Driver::SetAlignmentMatrix(unsigned int sensor_index, CmtMatrix alignment_matrix)
    {
        this->v_sensors[sensor_index].alignment_matrix = alignment_matrix;
    }

    bool Driver::DoHardwareScan()
    {
        XsensResultValue    res;
        List<CmtPortInfo>   port_info;
        unsigned long       port_count = 0;
        
        std::cout << "Scanning for connected Xsens devices..." << std::endl;
        xsens::cmtScanPorts(port_info);
        port_count = port_info.length();
        std::cout << "done" << std::endl;
        
        if (port_count == 0)
        {
            std::cout << "No motion trackers found" << std::endl;
            return false;
        }
        
        for (int i = 0; i < (int)port_count; i++)
        {
            std::cout << "Using COM port " << port_info[i].m_portName << " at ";
            switch (port_info[i].m_baudrate) 
            {
                case B9600:
                    std::cout << "9k6";
                    break;
                case B19200:
                    std::cout << "19k2";
                    break;
                case B38400:
                    std::cout << "38k4";
                    break;
                case B57600:
                    std::cout << "57k6";
                    break;
                case B115200:
                    std::cout << "115k2";
                    break;
                case B230400:
                    std::cout << "230k4";
                    break;
                case B460800:
                    std::cout << "460k8";
                    break;
                case B921600:
                    std::cout << "921k6";
                    break;
                default:
                    std::cout << port_info[i].m_baudrate;
            }
            std::cout << " baud" << std::endl;
        }
        std::cout << "Opening ports...";
        
        // open the port which the device is connected to and connect at the device's baudrate.
        
        for (int p = 0; p < (int)port_count; p++)
        {
            res = this->cmt3.openPort(port_info[p].m_portName,
                                      port_info[p].m_baudrate);
            if (res != XRV_OK)
            {
                std::cout << "ERROR: cmtOpenPort" << std::endl;
                return false;
            }
        }
        std::cout << "Done" << std::endl;
        
        // set the measurement timeout to 100 ms (default is 16 ms)

        int timeout = 100;
        res = this->cmt3.setTimeoutMeasurement(timeout);
        if (res != XRV_OK)
        {
            std::cout << "ERROR: set measurement timeout" << std::endl;
            return false;
        }
        std::cout << "Timeout set to " << timeout << " ms" << std::endl;
        
        // get the MT sensor count
        
        std::cout << "Retrieving MT count (excluding attached Xbus Master(s))" << std::endl;
        this->mt_count = this->cmt3.getMtCount();
        std::cout << "MT count: " << this->mt_count << std::endl;    
        
        this->v_sensors.resize(this->mt_count);
        
        // retrieve the device IDs

        std::cout << "Retrieving MT device IDs" << std::endl;
        for (unsigned int j = 0; j < this->mt_count; j++)
        {
            // res = this->cmt3.getDeviceId((unsigned char)(j+1), this->device_ids[j]);
            res = this->cmt3.getDeviceId((unsigned char)(j+1), this->v_sensors[j].device_id);
            if (res != XRV_OK)
            {
                std::cout << "ERROR: get device id" << std::endl;
                return false;
            }
        }
        
        return true;
    }

    bool Driver::SetConfiguration()
    {
        XsensResultValue res;
        
        // set the sensor to config state
        
        res = this->cmt3.gotoConfig();
        if (res != XRV_OK)
        {
            ROS_ERROR("ERROR: go to config");
            return false;
        }
        
        unsigned short sample_freq;
        sample_freq = this->cmt3.getSampleFrequency();
        
        // set the device output mode for the devices
        
        CmtOutputSettings settings;
        if ((this->output_mode & CMT_OUTPUTMODE_ORIENT) == 0)
        {
            this->output_settings = 0;
        }        
        settings = this->output_settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
        
        std::cout << "Configuring your mode selection" << std::endl;
        for (unsigned int i = 0; i < this->mt_count; i++)
        {
            CmtDeviceMode device_mode(this->output_mode, 
                                      settings, 
                                      sample_freq);
            if ((this->v_sensors[i].device_id & 0xFFF00000) != 0x00500000) 
            {
			    // not an MTi-G, remove all GPS related stuff
			    device_mode.m_outputMode &= 0xFF0F;
		    }
		    res = this->cmt3.setDeviceMode(device_mode, true, this->v_sensors[i].device_id);
		    if (res != XRV_OK)
		    {
		        std::cout << "ERROR: set device mode" << std::endl;
		        return false;
		    }
        }

        // Set aligment Matrix
        for(unsigned int i = 0; i < this->mt_count; i++)
        { 
            if(this->cmt3.setObjectAlignmentMatrix(this->v_sensors[i].alignment_matrix, this->v_sensors[i].device_id) != XRV_OK)
            {
                ROS_ERROR("Could not set alignment matrix for object %d", i);
                return false;
            }
            else
            {
                ROS_INFO("Alignment matrix set for object %d to M", i);
            }
        }

        res = this->cmt3.gotoMeasurement();
        if (res != XRV_OK)
        {
            ROS_ERROR("ERROR: go to measurement");
            return false;
        }
        
        return true;
    }

    bool Driver::Initialize()
    {
        if (this->DoHardwareScan() == false)
        {
            std::cout << "ERROR: DoHardwareScan()" << std::endl;
            this->cmt3.closePort();
            return false;
        }
        
        if (this->mt_count == 0)
        {
            std::cout << "No IMUs found" << std::endl;
            this->cmt3.closePort();
            return false;
        }
        
        if (this->SetConfiguration() == false)
        {
            std::cout << "ERROR: SetConfiguration()" << std::endl;
            return false;
        }
        
        this->lp_packet = new Packet((unsigned short)this->mt_count, this->cmt3.isXm());
        std::cout << "Everything is OK. Retrieving data..." << std::endl;
        
        return true;
    }

    bool Driver::SpinOnce()
    {        
        XsensResultValue res = this->cmt3.waitForDataMessage(this->lp_packet);
        if (res != XRV_OK)
        {
            if ((res == XRV_TIMEOUTNODATA) || (res == XRV_TIMEOUT))
            {
                return true;
            }
            
            delete this->lp_packet;
            this->cmt3.closePort();
            std::cout << "ERROR: " << (int)res << " ocurred in waitForDataMessage. Can not recover." << std::endl;
            return false;        
        }
        
        this->sample_data = this->lp_packet->getSampleCounter();
        if (this->RetrieveData() == false)
        {
            std::cout << "ERROR: RetrieveData()" << std::endl;
            return false;
        }
        
        return true;
    }

    bool Driver::RetrieveData()
    {
        for (unsigned int i = 0; i < this->mt_count; i++)
        {
            if ((this->output_mode & CMT_OUTPUTMODE_RAW) != 0)
            {
                this->v_sensors[i].raw_data.m_acc  = this->lp_packet->getRawAcc(i);
                this->v_sensors[i].raw_data.m_gyr  = this->lp_packet->getRawGyr(i);
                this->v_sensors[i].raw_data.m_mag  = this->lp_packet->getRawMag(i);
                this->v_sensors[i].raw_data.m_temp = this->lp_packet->getRawTemp(i);
                continue;
            }
            
            if ((this->output_mode & CMT_OUTPUTMODE_TEMP) != 0)
            {
                this->v_sensors[i].temperature_data = this->lp_packet->getTemp(i);
            }
            
            if ((this->output_mode & CMT_OUTPUTMODE_CALIB) != 0)
            {
                this->v_sensors[i].calibrated_data = this->lp_packet->getCalData(i);
            }
            
            if ((this->output_mode & CMT_OUTPUTMODE_ORIENT) == 0) 
		    {
			    continue;
		    }
		
		    switch (this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) 
		    {
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				    this->v_sensors[i].quaternion_data = this->lp_packet->getOriQuat(i);
				    break;
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				    this->v_sensors[i].euler_data = this->lp_packet->getOriEuler(i);
				    break;
			    case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				    this->v_sensors[i].matrix_data = this->lp_packet->getOriMatrix(i);
				    break;
			    default:
				    break;
		    }
		
		    /*if ((this->output_mode & CMT_OUTPUTMODE_POSITION) != 0) 
		    {
			    if (this->lp_packet->containsPositionLLA()) 
			    {
				    CmtVector positionLLA = this->lp_packet->getPositionLLA();
				    if (this->result_value != XRV_OK) 
				    {
					    std::cout << "ERROR: get position LLA" << std::endl;
				    }
	
				    for (int i = 0; i < 2; i++) 
				    {
					    double deg = positionLLA.m_data[i];
					    double min = (deg - (int)deg)*60;
					    double sec = (min - (int)min)*60;
				    }
			    } 
			    else 
			    {
				    std::cout << "No position data available" << std::endl;
			    }
		    }*/
        }
        
        return true;
    }

    int Driver::GetMtCount()
    {
        return (int)this->mt_count;
    }

    CmtOutputMode Driver::GetOutputMode()
    {
        return this->output_mode;
    }

    CmtOutputSettings Driver::GetOutputSettings()
    {
        return this->output_settings;
    }

    CmtQuat& Driver::GetQuatData(int mt_index)
    {
        return this->v_sensors[mt_index].quaternion_data;
    }

    CmtRawData& Driver::GetRawData(int mt_index)
    {
        return this->v_sensors[mt_index].raw_data;
    }

    CmtCalData& Driver::GetCalData(int mt_index)
    {
        return this->v_sensors[mt_index].calibrated_data;
    }

}
