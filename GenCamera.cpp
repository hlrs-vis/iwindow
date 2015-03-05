/*
 * MyGrabber.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: hpcthobs
 */

#include "GenCamera.h"

#include <ueye.h>
#include <iostream>
#include <stdlib.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>


#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;


GenCamera::GenCamera() {
	// TODO Auto-generated constructor stub
	get_success=false;
	DeviceNumber=0;
}

GenCamera::~GenCamera() {
	// TODO Auto-generated destructor stub

}

bool GenCamera::setCameraType(CameraType type)
{
	this->type= type;
	return 0;
}


/*bool MyGrabber::initAllCameras()
{
	this->initAllV4LCameras();
	this->initAllUEyeCameras();

	return 0;
}

bool MyGrabber::setNumberAllCameras()
{
	this->Number_All_Cameras= Number_UEye_Cameras+Number_V4L_Cameras;
	return 0;
}*/
