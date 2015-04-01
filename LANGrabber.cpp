/*
 * LANGrabber.cpp
 *
 *  Created on: Mar 9, 2015
 *      Author: hpcascha
 */

#include "LANGrabber.h"
#include <iostream>


LANGrabber::LANGrabber() {
	// TODO Auto-generated constructor stub
//	DeviceNumber = 0;
}

LANGrabber::~LANGrabber() {
	// TODO Auto-generated destructor stub
}

bool LANGrabber::setDeviceNumber(int DeviceNumber)
{
	this->DeviceNumber= DeviceNumber;
	return 0;
}

bool LANGrabber::setAddress(string address)
{
	this->address = address;
	return 0;
}

bool LANGrabber::initCamera()
{
		const string url = address;
		VideoCapture* cap0 = new VideoCapture(url);

		if (!cap0->isOpened())  // if not success, exit program
		{
			cout << "Cannot open the video camera, setting NumberLANCameras to: " << DeviceNumber << endl;
			//this->setNumberCameras(device);
			return -1;
		}
		else
		{

			double dWidth = cap0->get(cv::CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
			double dHeight = cap0->get(cv::CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

			cout << "Frame size : " << dWidth << " x " << dHeight << endl;

			this->LANcapture=(*cap0);

		}
		delete cap0;


	return true;
}

bool LANGrabber::getFrame()
{
	LANcapture.open(address);
	get_success = LANcapture.read(originalframe); // read a new frame from video
	//doesn't work after the first call for images, need to reopen cap0
//	resize(originalframe, originalframe, Size(1280,720)); //if 1280x720 is needed
	return get_success;
}

bool LANGrabber::close()
{
	return 0;
}
