/*
 * V4LGrabber.h
 *
 *  Created on: Jan 13, 2015
 *      Author: hpcthobs
 */

#ifndef V4LGRABBER_H_
#define V4LGRABBER_H_

#include "GenCamera.h"
using namespace std;
using namespace cv;


class V4LGrabber: public GenCamera {
public:
	V4LGrabber();
	virtual ~V4LGrabber();

	//V4L Group
	//bool setNumberCameras(int Number_V4L_Cameras);
	//bool initAllCameras();

	bool setDeviceNumber(int DeviceNumber);
	bool initCamera();
	bool getFrame();
	//bool getAllFrames();
	bool close();


	VideoCapture V4Lcapture;

};

#endif /* V4LGRABBER_H_ */
