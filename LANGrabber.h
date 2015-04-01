/*
 * LANGrabber.h
 *
 *  Created on: Mar 9, 2015
 *      Author: hpcascha
 */

#ifndef LANGRABBER_H_
#define LANGRABBER_H_

#include "GenCamera.h"
using namespace std;
using namespace cv;

class LANGrabber: public GenCamera{
public:
	LANGrabber();
	virtual ~LANGrabber();

	bool setDeviceNumber(int DeviceNumber);
	bool setAddress(string address);
	bool initCamera();
	bool getFrame();
	bool close();

	VideoCapture LANcapture;
};

#endif /* LANGRABBER_H_ */
