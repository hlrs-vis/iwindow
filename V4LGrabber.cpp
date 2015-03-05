/*
 * V4LGrabber.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: hpcthobs
 */

#include "V4LGrabber.h"
#include <iostream>


V4LGrabber::V4LGrabber() {
	// TODO Auto-generated constructor stub
	DeviceNumber=0;
}

V4LGrabber::~V4LGrabber() {
	// TODO Auto-generated destructor stub
}

bool V4LGrabber::setDeviceNumber(int DeviceNumber)
{
	this->DeviceNumber= DeviceNumber;
	return 0;
}

bool V4LGrabber::initCamera()
{

		VideoCapture* cap0 = new VideoCapture(this->DeviceNumber);

		if (!cap0->isOpened())  // if not success, exit program
		{
			cout << "Cannot open the video camera, setting NumberV4LCameras to: " << DeviceNumber << endl;
			//this->setNumberCameras(device);
			return -1;
		}
		else
		{
			cap0->set(cv::CAP_PROP_FRAME_WIDTH, 1280); // valueX = your wanted width
			cap0->set(cv::CAP_PROP_FRAME_HEIGHT, 720); // valueY = your wanted heigth
			cap0->set(cv::CAP_PROP_FPS,30);

			double dWidth = cap0->get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
			double dHeight = cap0->get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

			cout << "Frame size : " << dWidth << " x " << dHeight << endl;

			this->V4Lcapture=(*cap0);
			//this->get_success.resize(get_success.size()+1);
			//this->originalframe.resize(originalframe.size()+1);
		}
		delete cap0;


	return true;

}

/*bool V4LGrabber::initAllCameras()
{
	for (int i=0; i<Number_Cameras; i++)
	{
		this->initCamera(i);
	}
	return true;
}*/

/*bool V4LGrabber::setNumberCameras(int Number_Cameras)
{
	this->Number_Cameras= Number_Cameras;
	//this->setNumberAllCameras();
	return 0;
}*/


/*bool V4LGrabber::getAllFrames()
{
	int i;
	//cout << "trying to read " << Number_V4L_Cameras << " devices" << endl;

	for (i=0; i<Number_Cameras; i++)
	{
		//cout << "trying to read device" << i << endl;
		get_success[i] = this->getFrame(i);
	}
	return 0;
}*/

bool V4LGrabber::getFrame()
{
	//cout << "trying to read device" << device << endl;
	get_success = V4Lcapture.read(originalframe); // read a new frame from video
	return get_success;
}

bool V4LGrabber::close()
{
	return 0;
}

