#include <xsens_driver/xsens_driver.h>
#include <dfv/dfv.h>
#include <xsens_driver/utils.h>

int main(int argc, char** argv)
{
	xsens::Driver driver;
	
	// Configuración del sensor
	driver.SetOutputMode(CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_TEMP);
	driver.SetOutputSettings(CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION);
	driver.SetAlignmentMatrix(0, xsens::DfvToCmtMatrix(dfv::Matrix::Identity(3)));
	driver.Initialize();	
	
	// Toma de datos
	while(driver.SpinOnce())
	{
		// Código para lectura y procesamiento de datos
		// ...
	}	
		
	return 0;
}
