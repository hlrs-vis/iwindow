/*
 * MyGrabber.h
 *
 *  Created on: Dec 16, 2014
 *      Author: hpcthobs
 */

#ifndef GENCAMERA_H_
#define GENCAMERA_H_


#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <ueye.h>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

enum CameraType {V4L, UEye, LAN};

class GenCamera {
public:
	GenCamera();
	virtual ~GenCamera();

	virtual bool setDeviceNumber(int DeviceNumber){return 0;};
	virtual bool setAddress(string address){return 0;}
	virtual bool initCamera(){return 0;};
	virtual bool getFrame(){return 0;};
	virtual bool close(){return 0;};
	bool setCameraType(CameraType type);


    Mat originalframe;
    bool get_success;
	int DeviceNumber;
	CameraType type;
	string address;




private:


};

#endif /* GENCAMERA_H_ */
