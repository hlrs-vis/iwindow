/*
 * UEyeGrabber.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: hpcthobs
 */

#include "UEyeGrabber.h"

#define CAPTURE_WIDTH  1280
#define CAPTURE_HEIGHT 1024


UEyeGrabber::UEyeGrabber() {
	hCam=0;
	// TODO Auto-generated constructor stub

}

UEyeGrabber::~UEyeGrabber() {
	// TODO Auto-generated destructor stub
		this->close();
}

bool UEyeGrabber::setDeviceNumber(HIDS hCam)
{
	this->hCam= hCam;
	cout << "device number: "<< hCam << endl;
	cout << "device number: "<< this->hCam << endl;
	DeviceNumber=this->hCam;


	return 0;
}


/*
bool UEyeGrabber::initAllCameras()
{
	for (int i=0; i<Number_Cameras; i++)
	{
		this->Cam.resize(Cam.size()+1);
		this->hCam.resize(hCam.size()+1);

		bool init_success=true;
		if (!(init_success=this->initCamera(&(this->hCam[i]),(void*)&(this->Cam[i]))))
		{
			cout << "init cam " << i << " error, setting Number_UEye_Cameras to " << i << endl;
			this->setNumberCameras(i);

			// run = false;
		}
		else
		{
			cout << "init cam " << i << " success" << endl;
			//this->get_success.resize(get_success.size()+1);

			Mat tempframe(Size(1280,1024),CV_8UC3);

			this->originalframe.push_back(tempmatrix);
		}
	}
	return 0;
}*/

bool UEyeGrabber::initCamera()
{	cout << "device number: "<< this->hCam << endl;

	bool result = true;
	INT is_ParameterSet(HIDS hCam, UINT nCommand, void* pParam, UINT cbSizeOfParam);

	//Initilization variables IDS uEye UI1220SE-C

	int BITS_PER_PIXEL = 24;
	int pWidth = CAPTURE_WIDTH;
	int pHeight = CAPTURE_HEIGHT;
	SENSORINFO sensor_info;
	CAMINFO camera_info;

	//memory pointer
	char* m_pcImageMemory;
	this->Cam.m_pcImageMemory_tmp = m_pcImageMemory;
	int m_lMemoryId;

	//Pulizia memoria da foto precedenti
	if (this->hCam != 0)
	{
		is_FreeImageMem (this->hCam,Cam.m_pcImageMemory_tmp,m_lMemoryId);
		is_ExitCamera(this->hCam);
	}

	//initialize camera
	int initcamera = is_InitCamera(&this->hCam, NULL);
	cout << "device number: "<< this->hCam << endl;

	if(initcamera != IS_SUCCESS)
	{
		cout<<endl<<"Initializing camera failed"<<endl;
		result = false;
	}

	// Get camera info
	int camerainfo = is_GetCameraInfo (this->hCam, &camera_info);
	if(camerainfo != IS_SUCCESS)
	{
		//printf("Cannot get camera info");
		result = false;
	}
	// Get sensor info
	int sensorinfo = is_GetSensorInfo (this->hCam, &sensor_info);
	if(sensorinfo != IS_SUCCESS)
	{
		//printf("Cannot get sensor info");
		result = false;
	}

	cout<<"Serial number: " << camera_info.SerNo << endl;


	//Output camera/sensor information
	//cout<<endl<<"<<< Camera details >>>"<<endl;
	//cout<<"Serial number: " << camera_info.SerNo << endl;
	//cout << "Producer: " << camera_info.ID << endl;
	//cout << "Model: " << sensor_info.strSensorName << endl;
	//cout << "Max image dimensions: " << sensor_info.nMaxWidth << "x" << sensor_info.nMaxHeight << endl << endl;


	//Set color mode BGR24
	int colormode = is_SetColorMode(this->hCam, IS_CM_BGR8_PACKED);
	//int colormode = is_SetColorMode(*hCam, IS_SET_CM_RGB24);
	if(colormode != IS_SUCCESS)
	{
		//printf("Cannot switch on color mode");
		result = false;
	}

	//Set image dimensions
	int pXPos = (sensor_info.nMaxWidth);
	int pYPos = (sensor_info.nMaxHeight);

	//Initialize memory
	int rit = is_AllocImageMem (this->hCam,pXPos,pYPos, 24, &Cam.m_pcImageMemory_tmp, &m_lMemoryId);
	if(rit != IS_SUCCESS)
	{
		cout<<endl<<"Cannot initialize memory"<<endl;
		result = false;
	}
	//cout<<endl<<"Memory initialized"<<endl;

	//Set Image Memory
	int rat = is_SetImageMem (this->hCam, Cam.m_pcImageMemory_tmp, m_lMemoryId);
	if(rat != IS_SUCCESS)
	{
		cout<<endl<<"Cannot set Image memory"<<endl;
		result = false;
	}
	//cout<<endl<<"memory activated"<<endl;

	//set color correction
	double strenght_factor = 1.0;
	int colorcorrection = is_SetColorCorrection(this->hCam, IS_CCOR_ENABLE, &strenght_factor);

	//Set white balance
	double pval = 1;
	int whiteb = is_SetAutoParameter(this->hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &pval, 0);

	//set gain
	double gval = 1;
	int gains = is_SetAutoParameter(this->hCam, IS_SET_ENABLE_AUTO_GAIN, &gval, 0);

	Mat tempframe(Size(1280,1024),CV_8UC3);
	tempframe.copyTo(this->originalframe);


	return result;
}

/*bool UEyeGrabber::getAllFrames()
{
	for (int j=0; j<(Number_Cameras); j++)
	{
		//get_success[j]=this->getFrame(&originalframe[j], &(hCam[j]), (void*)&Cam[j]); // read a new frame from video
	}
	return 0;
}*/


bool UEyeGrabber::getFrame()
{


	int dummy;
	char *pMem, *pLast;
	//	HIDS hCam=cameraID;

	int sho = is_FreezeVideo(hCam, IS_WAIT);
	//int sho = is_CaptureVideo(*hCam, IS_WAIT);
	if(sho == IS_SUCCESS)
	{
		//cout<<endl<<"retrieved images"<<endl;
		get_success=true;
	}

	if(sho != IS_SUCCESS)
	{
		cout<<endl<<"Cannot retrieve images"<<endl;
		get_success=false;
	}

	if (sho == IS_SUCCESS)
	{
		int m_Ret = is_GetActiveImageMem(hCam, &pLast, &dummy);
		int n_Ret = is_GetImageMem(hCam, (void**)&pLast);

		memcpy(originalframe.data, Cam.m_pcImageMemory_tmp, 1280 * 1024 * 3);
	}


	return get_success;
}

bool UEyeGrabber::close()
{
	int en = is_ExitCamera(this->hCam);
	if (en == IS_SUCCESS)
	{
		cout<<"Ueye Camera " << this->hCam << " closed correctly"<<endl;
	}
	return en;
}
