/*
 * CameraManager.h
 *
 *  Created on: Jan 15, 2015
 *      Author: hpcthobs
 */

#ifndef CAMERAMANAGER_H_
#define CAMERAMANAGER_H_

#include "GenCamera.h"
#include "V4LGrabber.h"
#include "UEyeGrabber.h"



class CameraManager {
public:
	CameraManager();
	virtual ~CameraManager();

	bool addCamera(int device, CameraType type);
	bool capture();

	vector<GenCamera*> Cameras;
};

#endif /* CAMERAMANAGER_H_ */
