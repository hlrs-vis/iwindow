/*
 * CameraManager.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: hpcthobs
 */

#include "CameraManager.h"

CameraManager::CameraManager() {
	// TODO Auto-generated constructor stub

}

CameraManager::~CameraManager() {
	// TODO Auto-generated destructor stub
}

bool CameraManager::addCamera(int device, CameraType type)
{
	GenCamera *newCamera= new GenCamera();
	switch (type)
	{
		case V4L:
		{
			newCamera= new V4LGrabber();
			break;
		}
		case UEye:
		{
			newCamera= new UEyeGrabber();
			break;
		}
	}

	newCamera->setDeviceNumber(device);
	newCamera->initCamera();
	Cameras.push_back(newCamera);

	return 0;
}

bool CameraManager::capture(){
	for (int i=0; i<(this->Cameras.size()); i++)
	{
		this->Cameras[i]->getFrame();
	}
	return 0;
}
