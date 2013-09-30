#include <xsens_driver/xsens_driver.h>

namespace xsens
{

    Driver::Driver():
        mt_count(0),
        output_mode(CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT),
        output_settings(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT),
        skip_factor(10),
        skip_factor_count(0),
        lp_packet(NULL)
    {
        //this->output_settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
        if(this->DoHardwareScan() == false)
        {
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            this->cmt3.closePort();
        }
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
    
    CmtOutputMode Driver::GetOutputMode() const
    {
        return this->output_mode;
    }
    
    void Driver::SetOutputSettings(CmtOutputSettings output_settings)
    {
        this->output_settings = (output_settings | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
    }
    
    CmtOutputSettings Driver::GetOutputSettings() const
    {
        //return (this->output_settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK);
        return this->output_settings;
    }
    
    void Driver::SetOutputMode(unsigned int index, CmtOutputMode output_mode)
    {
        this->v_sensors[index].SetOutputMode(output_mode);
    }
    
    CmtOutputMode Driver::GetOutputMode(unsigned int index) const
    {
        return this->v_sensors[index].GetOutputMode();
    }
    
    void Driver::SetOutputSettings(unsigned int index, CmtOutputSettings output_settings)
    {
        this->v_sensors[index].SetOutputSettings(output_settings);
    }
    
    CmtOutputSettings Driver::GetOutputSettings(unsigned int index) const
    {
        return this->v_sensors[index].GetOutputSettings();
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
        
        ROS_INFO("Scanning for connected Xsens devices...");
        xsens::cmtScanPorts(port_info);
        port_count = port_info.length();
        ROS_INFO("Scanning done");
        
        if (port_count == 0)
        {
            ROS_ERROR("No motion trackers found");
            return false;
        }
        
        for (int i = 0; i < (int)port_count; i++)
        {
            std::stringstream ss;
            ss << "Using COM port " << port_info[i].m_portName << " at ";
            switch (port_info[i].m_baudrate) 
            {
                case B9600:
                    ss << "9k6";
                    break;
                case B19200:
                    ss << "19k2";
                    break;
                case B38400:
                    ss << "38k4";
                    break;
                case B57600:
                    ss << "57k6";
                    break;
                case B115200:
                    ss << "115k2";
                    break;
                case B230400:
                    ss << "230k4";
                    break;
                case B460800:
                    ss << "460k8";
                    break;
                case B921600:
                    ss << "921k6";
                    break;
                default:
                    ss << port_info[i].m_baudrate;
            }
            ss << " baud" << std::endl;
            ROS_INFO("%s", ss.str().c_str());
        }
        ROS_INFO("Opening ports...");
        
        // open the port which the device is connected to and connect at the device's baudrate.
        
        for (int p = 0; p < (int)port_count; p++)
        {
            res = this->cmt3.openPort(port_info[p].m_portName,
                                      port_info[p].m_baudrate);
            if (res != XRV_OK)
            {
                ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
                return false;
            }
        }
        std::cout << "Done" << std::endl;
        
        // set the measurement timeout to 100 ms (default is 16 ms)

        int timeout = 100;
        res = this->cmt3.setTimeoutMeasurement(timeout);
        if (res != XRV_OK)
        {
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            return false;
        }
        ROS_INFO("Timeout set to %d ms", timeout);
        
        // get the MT sensor count
        
        ROS_INFO("Retrieving MT count (excluding attached Xbus Master(s))");
        this->mt_count = this->cmt3.getMtCount();
        ROS_INFO("MT count: %d", this->mt_count);
        
        this->v_sensors.resize(this->mt_count);
        
        // retrieve the device IDs

        ROS_INFO("Retrieving MT device IDs");
        for (unsigned int j = 0; j < this->mt_count; j++)
        {
            // res = this->cmt3.getDeviceId((unsigned char)(j+1), this->device_ids[j]);
            res = this->cmt3.getDeviceId((unsigned char)(j+1), this->v_sensors[j].device_id);
            if (res != XRV_OK)
            {
                ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
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
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            ROS_ERROR("Could not go to configuration mode");
            return false;
        }
        
        unsigned short sample_freq;
        sample_freq = this->cmt3.getSampleFrequency();
        
        // set the device output mode for the devices
        
        // if there's no orient data
        /*if ((this->output_mode & CMT_OUTPUTMODE_ORIENT) == 0)
        {
            this->output_settings = 0;
            this->output_settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
        } */       
        
        
        ROS_INFO("Configuring your mode selection");
        for (unsigned int i = 0; i < this->mt_count; i++)
        {
            //CmtDeviceMode device_mode(this->output_mode, 
                                      //this->output_settings, 
                                      //sample_freq);
            if ((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_ORIENT) == 0)
            {
                //this->output_settings = 0;
                //this->output_settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
                this->v_sensors[i].SetOutputSettings(CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
            }
            CmtDeviceMode device_mode(this->v_sensors[i].GetOutputMode(), 
                                      this->v_sensors[i].GetOutputSettings(), 
                                      sample_freq);
            if ((this->v_sensors[i].device_id & 0xFFF00000) != 0x00500000) 
            {
			    // not an MTi-G, remove all GPS related stuff
			    device_mode.m_outputMode &= 0xFF0F;
		    }
		    res = this->cmt3.setDeviceMode(device_mode, true, this->v_sensors[i].device_id);
		    if (res != XRV_OK)
		    {
		        ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
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
        /*if (this->DoHardwareScan() == false)
        {
            std::cout << "ERROR: DoHardwareScan()" << std::endl;
            this->cmt3.closePort();
            return false;
        }*/
        
        if (this->mt_count == 0)
        {
            //ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            ROS_ERROR("No Imus found.");
            this->cmt3.closePort();
            return false;
        }
        
        if (this->SetConfiguration() == false)
        {
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            return false;
        }
        
        this->lp_packet = new Packet((unsigned short)this->mt_count, this->cmt3.isXm());
        ROS_INFO("Everything is OK. Retrieving data...");
        
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
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
            return false;        
        }
        
        this->sample_data = this->lp_packet->getSampleCounter();
        if (this->RetrieveData() == false)
        {
            //std::cout << "ERROR: RetrieveData()" << std::endl;
            ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
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
            
            if ((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_TEMP) != 0)
            {
                this->v_sensors[i].temperature_data = this->lp_packet->getTemp(i);
            }
            
            if ((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_CALIB) != 0)
            {
                this->v_sensors[i].calibrated_data = this->lp_packet->getCalData(i);
            }
            
            if ((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_POSITION) != 0) 
		    {
			    if (this->lp_packet->containsPositionLLA(i)) 
			    {
				    //CmtVector positionLLA = this->lp_packet->getPositionLLA();
				    this->v_sensors[i].position_lla = this->lp_packet->getPositionLLA(i);
				    /*if (this->result_value != XRV_OK) 
				    {
					    std::cout << "ERROR: get position LLA" << std::endl;
				    }*/
	
				    /*for (int i = 0; i < 2; i++) 
				    {
					    double deg = positionLLA.m_data[i];
					    double min = (deg - (int)deg)*60;
					    double sec = (min - (int)min)*60;
				    }*/
			    } 
			    else
			    {
			        ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
			        ROS_ERROR("No PositionLLA data available");
			    }
		    }
		    
		    if((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_GPSPVT_PRESSURE) != 0)
		    {
		        if (this->lp_packet->containsGpsPvtData(i)) 
			    {
				    this->v_sensors[i].gps_pvt_data = this->lp_packet->getGpsPvtData(i);
				    // ROS_INFO("Retrieving GPS pvt Data");
			    }
			    else
			    {
			        ROS_ERROR("In function %s at line %d in file %s", __PRETTY_FUNCTION__, __LINE__, __FILE__);
			        ROS_ERROR("No GpsPvt data available");
			    }    
		    }
            
            if ((this->v_sensors[i].GetOutputMode() & CMT_OUTPUTMODE_ORIENT) == 0) 
		    {
			    continue;
		    }
		
		    switch (this->v_sensors[i].GetOutputSettings() & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) 
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
		    
        }
        
        return true;
    }

    unsigned int Driver::GetMtCount()
    {
        return this->mt_count;
    }

    CmtOutputMode Driver::GetOutputMode()
    {
        return this->output_mode;
    }

    CmtOutputSettings Driver::GetOutputSettings()
    {
        return this->output_settings;
    }

    CmtQuat& Driver::GetOriQuat(int mt_index)
    {
        return this->v_sensors[mt_index].quaternion_data;
    }
    
    CmtMatrix& Driver::GetOriMatrix(int mt_index)
    {
        return this->v_sensors[mt_index].matrix_data;
    }
    
    CmtEuler& Driver::GetOriEuler(int mt_index)
    {
        return this->v_sensors[mt_index].euler_data;
    }

    CmtRawData& Driver::GetRawData(int mt_index)
    {
        return this->v_sensors[mt_index].raw_data;
    }

    CmtCalData& Driver::GetCalData(int mt_index)
    {
        return this->v_sensors[mt_index].calibrated_data;
    }
    
    CmtVector& Driver::GetPositionLLA(int mt_index)
    {
        return this->v_sensors[mt_index].position_lla;
    }
    
    CmtGpsPvtData& Driver::GetGpsPvtData(int mt_index)
    {
        return this->v_sensors[mt_index].gps_pvt_data;
    }

}
