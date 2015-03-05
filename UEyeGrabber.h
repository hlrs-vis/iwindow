/*
 * UEyeGrabber.h
 *
 *  Created on: Jan 13, 2015
 *      Author: hpcthobs
 */

#ifndef UEYEGRABBER_H_
#define UEYEGRABBER_H_
#include <iostream>

#include "GenCamera.h"
using namespace std;
using namespace cv;

struct Ueye_info
{
	char* m_pcImageMemory_tmp;
};

class UEyeGrabber : public GenCamera{
public:
	UEyeGrabber();
	~UEyeGrabber();

	//Ueye group
	bool setDeviceNumber(HIDS hCam);
	bool initCamera();
	bool getFrame();
	//bool getAllFrames();
	bool close();

	/*
	bool setNumberCameras(int Number_UEye_Cameras);
	bool initAllCameras();
	bool initCamera(HIDS* hCam, void* param);
	bool getAllFrames();
	bool getFrame(Mat* image, HIDS* hCam, void* param);
	bool close(HIDS hCam);*/

	Ueye_info Cam;
	HIDS hCam;

};

#endif /* UEYEGRABBER_H_ */

