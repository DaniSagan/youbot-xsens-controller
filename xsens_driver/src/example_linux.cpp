#include <stdio.h>		
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <curses.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include "example_linux.h"

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printf("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace xsens;                                   

int main(int argc, char* argv[])
{
	(void) argc; (void) argv;	// Make the compiler stop complaining about unused parameters
	xsens::Cmt3 cmt3;
	unsigned long mtCount = 0;
	CmtDeviceId deviceIds[256];
	
	CmtOutputMode mode;
	CmtOutputSettings settings;

	XsensResultValue res = XRV_OK;
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;

	// Perform hardware scan
	mtCount = doHardwareScan(cmt3, deviceIds);
	
	if (mtCount == 0) {
		printf("No IMUs found. Quitting...\n");
		cmt3.closePort();
		return 0;
	}
	
	mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;
/*      mode = CMT_OUTPUTMODE_CALIB;
		mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        mode = CMT_OUTPUTMODE_RAW;*/
    settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
/*      if ((mode & CMT_OUTPUTMODE_ORIENT) != 0)
		settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
		settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
		settings = CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX;
	    else 
		settings = 0;
	    }*/
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

	// Set device to user input settings
	doMtSettings(cmt3, mode, settings, deviceIds);

	// vars for sample counter & temp.
	unsigned short sdata;
	double tdata;
	
	//structs to hold data.
	CmtCalData cal_data;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
    CmtRawData raw_data;

	// Initialize packet for data
	Packet* packet = new Packet((unsigned short) mtCount, cmt3.isXm());
	
	// ROS	
	ros::init(argc, argv, "MT_publisher");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
	
	// main loop
	while (res == XRV_OK) 
	{
		XsensResultValue result = cmt3.waitForDataMessage(packet);
		if(result != XRV_OK)
		{
			if( (result == XRV_TIMEOUTNODATA) || (result == XRV_TIMEOUT) )
				continue;  //Ignore the error and restart the while loop			

			delete packet;
			cmt3.closePort();
			printf("\nError %d occured in waitForDataMessage, can not recover.\n", result);
			exit(1);
		}

		//get sample count, goto position & display.
		sdata = packet->getSampleCounter();
		//mvprintw(y, x, "Sample Counter %05hu\n", sdata);

		if (screenSkipFactorCnt++ != screenSkipFactor) 
		{
			continue;
		}
		screenSkipFactorCnt = 0;
		
		for (unsigned int i = 0; i < mtCount; i++) 
		{			
			if ((mode & CMT_OUTPUTMODE_RAW) != 0) 
			{
		        raw_data.m_acc = packet->getRawAcc(i);	    
		        raw_data.m_gyr = packet->getRawGyr(i);
		        raw_data.m_mag = packet->getRawMag(i);
		        raw_data.m_temp = packet->getRawTemp(i);

		        /*mvprintw(y + 4, 0, "%u\t%u\t%u", 
		            raw_data.m_acc.m_data[0], raw_data.m_acc.m_data[1], raw_data.m_acc.m_data[2]);
				mvprintw(y + 6, 0, "%u\t%u\t%u", 
				    raw_data.m_gyr.m_data[0], raw_data.m_gyr.m_data[1], raw_data.m_gyr.m_data[2]);			        
				mvprintw(y + 8, 0, "%u\t%u\t%u", 
				    raw_data.m_mag.m_data[0], raw_data.m_mag.m_data[1], raw_data.m_mag.m_data[2]);	
				mvprintw(y + 10, 0, "%u", raw_data.m_temp);*/
				
				continue;	
			}
			
			// Output Temperature
			if ((mode & CMT_OUTPUTMODE_TEMP) != 0) 
			{					
				tdata = packet->getTemp(i);
				//mvprintw(y + 4 + i * screenSensorOffset, x, "%6.2f", tdata);
			}
			//move(y + 5 + temperatureOffset + i * screenSensorOffset, x);
			if ((mode & CMT_OUTPUTMODE_CALIB) != 0) 
			{					
				cal_data = packet->getCalData(i);
				/*mvprintw(y + 5 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", 	caldata.m_acc.m_data[0], 
						caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
				mvprintw(y + 7 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", caldata.m_gyr.m_data[0], 
						caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2]);
				mvprintw(y + 9 + temperatureOffset + i * screenSensorOffset, x,
						"%6.2f\t%6.2f\t%6.2f",caldata.m_mag.m_data[0], 
						caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
				move(y + 13 + temperatureOffset + i * screenSensorOffset, x);*/
			}

			if ((mode & CMT_OUTPUTMODE_ORIENT) == 0) 
			{
				continue;
			}

			switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			    {
				// Output: quaternion
				qat_data = packet->getOriQuat(i);
				sensor_msgs::Imu imu_msg;
				imu_msg.header.stamp = ros::Time::now();
				imu_msg.orientation.w = qat_data.m_data[0];
				imu_msg.orientation.x = qat_data.m_data[1];
				imu_msg.orientation.y = qat_data.m_data[2];
				imu_msg.orientation.z = qat_data.m_data[3];
				imu_pub.publish(imu_msg);
				ros::spinOnce();
				/*mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\t%6.3f", qat_data.m_data[0], 
						qat_data.m_data[1], qat_data.m_data[2], qat_data.m_data[3]);*/
				}
				break;

			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				// Output: Euler
				euler_data = packet->getOriEuler(i);
				/*mvprintw(yt++, xt, "%6.1f\t%6.1f\t%6.1f", euler_data.m_roll, 
						euler_data.m_pitch, euler_data.m_yaw);*/
				break;

			case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				// Output: Cosine Matrix
				matrix_data = packet->getOriMatrix(i);
				/*mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f", matrix_data.m_data[0][0], 
						matrix_data.m_data[0][1], matrix_data.m_data[0][2]);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[1][0], 
						matrix_data.m_data[1][1], matrix_data.m_data[1][2]);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[2][0], 
						matrix_data.m_data[2][1], matrix_data.m_data[2][2]);*/
				break;
			default:
				break;
			}

			if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
				if (packet->containsPositionLLA()) {
					/* output position */
					CmtVector positionLLA = packet->getPositionLLA();
					if (res != XRV_OK) {
						// printw("error %ud", res);
					}
	
					for (int i = 0; i < 2; i++) {
						double deg = positionLLA.m_data[i];
						double min = (deg - (int)deg)*60;
						double sec = (min - (int)min)*60;
						// printw("%3d\xb0%2d\'%2.2lf\"\t", (int)deg, (int)min, sec);
					}
					// printw(" %3.2lf\n", positionLLA.m_data[2]);
				} else {
					// printw("No position data available\n");
				}
			}
		}	
	}

	delete packet;
	cmt3.closePort();
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	int mtCount;
	
	printf("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	printf("done\n");

	if (portCount == 0) {
		printf("No MotionTrackers found\n\n");
		return 0;
	}

	for(int i = 0; i < (int)portCount; i++) {	
		printf("Using COM port %s at ", portInfo[i].m_portName);
		
		switch (portInfo[i].m_baudrate) {
		case B9600  : printf("9k6");   break;
		case B19200 : printf("19k2");  break;
		case B38400 : printf("38k4");  break;
		case B57600 : printf("57k6");  break;
		case B115200: printf("115k2"); break;
		case B230400: printf("230k4"); break;
		case B460800: printf("460k8"); break;
		case B921600: printf("921k6"); break;
		default: printf("0x%lx", portInfo[i].m_baudrate);
		}
		printf(" baud\n\n");
	}

	printf("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	printf("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);

	//get the Mt sensor count.
	printf("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	// printw("MotionTracker count: %d\n\n", mtCount);
	printf("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs 
	// printw("Retrieving MotionTrackers device ID(s)\n");
	printf("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		// printw("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
		printf("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode, 
		CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	printf("Configuring your mode selection\n");

	for (unsigned int i = 0; i < mtCount; i++) {
		CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
			// not an MTi-G, remove all GPS related stuff
			deviceMode.m_outputMode &= 0xFF0F;
		}
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");
}

