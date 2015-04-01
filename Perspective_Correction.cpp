
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <opencv/highgui.h>

#include <sys/time.h>
#include <ueye.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "CameraManager.h"



struct Mouse_info {
	cv::Point pt1, pt2, pt3; bool RIGHTBUTTONUP;
	Mouse_info(){
		pt1.x=500;
		pt1.y=500;
	}
	};

struct PositionedMask {
	cv::Mat Mask;
	Rect BBox;
};


void MouseEvent(int event, int x, int y, int flags, void* param)
{
	if  ( event == EVENT_LBUTTONDOWN )
		{
		//cout << "Left button of the mouse is down - position (" << x << ", " << y << ")" << endl;
		Mouse_info* ptPtr = (Mouse_info*)param;
		        ptPtr->pt1.x = x;
		        ptPtr->pt1.y = y;
		        ptPtr->pt2.x = 0;
		        ptPtr->pt2.y = 0;
   		}
	else if  ( event == EVENT_LBUTTONUP )
		{
		//cout << "Left button of the mouse is up - position (" << x << ", " << y << ")" << endl;
		Mouse_info* ptPtr = (Mouse_info*)param;
		        ptPtr->pt1.x = x;
		        ptPtr->pt1.y = y;
		        ptPtr->pt2.x = 0;
		        ptPtr->pt2.y = 0;
		}
	else if  ( event == EVENT_RBUTTONDOWN )
		{
		//cout << "Left button of the mouse is up - position (" << x << ", " << y << ")" << endl;
		Mouse_info* ptPtr = (Mouse_info*)param;
        		ptPtr->pt3.x = 0;
        		ptPtr->pt3.y = 0;
				ptPtr->pt1.x = x;
		        ptPtr->pt1.y = y;
		        ptPtr->pt2.x = 0;
		        ptPtr->pt2.y = 0;
		}
	else if  ( event == EVENT_RBUTTONUP )
		{
		//cout << "Left button of the mouse is up - position (" << x << ", " << y << ")" << endl;
		Mouse_info* ptPtr = (Mouse_info*)param;
        		ptPtr->RIGHTBUTTONUP = 1;
		}
    else if ( event == EVENT_MOUSEMOVE )
    {


    	if ( (flags & EVENT_FLAG_LBUTTON) == EVENT_FLAG_LBUTTON)
    	{
    		//cout << "Mouse move over the window while left button pressed - position (" << x << ", " << y << ")" << endl;

    		Mouse_info* ptPtr = (Mouse_info*)param;
    		ptPtr->pt2.x = x-ptPtr->pt1.x;
    		        ptPtr->pt2.y = y-ptPtr->pt1.y;
    		        ptPtr->pt1.x = x;
    		        ptPtr->pt1.y = y;
    	}
    	if ( (flags & EVENT_FLAG_RBUTTON) == EVENT_FLAG_RBUTTON)
    	{
    		//cout << flags << endl;
    		//cout << "Mouse move over the window while right button pressed - position (" << x << ", " << y << ")" << endl;

    		Mouse_info* ptPtr = (Mouse_info*)param;
	        ptPtr->pt3.x = x;//-ptPtr->pt1.x;
	        ptPtr->pt3.y = y;//-ptPtr->pt1.y;
	        //ptPtr->pt1.x = x;
	        //ptPtr->pt1.y = y;
    	}
    	else
    	{
    		//cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    	}
    }
}

Mat drawPolygon(Mat outputframe ,std::vector<cv::Point2i> polygoncorners, bool closedpolygon, bool filled, Scalar fillcolor)
{
    const Scalar RED(0,0,255), GREEN(0,255,0), BLUE(255,0,0);

	// Add circles around corner points
	if (polygoncorners.size()>=1)
	{
	for (int i=0;i<polygoncorners.size();i++)
		{
		circle(outputframe,polygoncorners[i],5, GREEN,2,8);
		}
	}

	// Add lines for shape
	if (polygoncorners.size()>=2)
	{
		for (int i=0;i<polygoncorners.size()-1;i++)
		{
			line(outputframe,polygoncorners[i],polygoncorners[i+1], GREEN,2,8);
		}
		if (closedpolygon)
		{
			line(outputframe,polygoncorners[polygoncorners.size()-1],polygoncorners[0], GREEN,2,8);
		}
	}
	if (filled==1)
	{

	}
	return outputframe;
}

Mat polygonfittingmask(Mat sampleframe, std::vector<cv::Point2f> polygoncorners)
{
	//Create fitting mask from corners
	// Draw polygon
	Mat maskfit1;
	maskfit1 = Mat::zeros(sampleframe.rows+2, sampleframe.cols+2, CV_8UC1);
	for (uint i=0;i<polygoncorners.size()-1;i++)
		{
		line(maskfit1,polygoncorners[i],polygoncorners[i+1], 255,1,8);
		}
	line(maskfit1,polygoncorners[polygoncorners.size()-1],polygoncorners[0], 255,1,8);

	/// Get the moments
	Moments mu;
	mu = moments( polygoncorners, false );
	///  Get the mass center:
    Point2i mc, mc_new;
	mc = Point2i( mu.m10/mu.m00 , mu.m01/mu.m00 );
	double test=pointPolygonTest(polygoncorners,mc,0);

	// Find point inside polygon

	// Find axis aligned bounding box
    int max_x,max_y,min_x,min_y;

	max_y=polygoncorners[0].y;
	min_y=polygoncorners[0].y;
	max_x=polygoncorners[0].x;
	min_x=polygoncorners[0].x;

	for (uint i=0; i < polygoncorners.size(); i++)
	{
		if (polygoncorners[i].x > max_x)
			max_x=polygoncorners[i].x;
		if (polygoncorners[i].y > max_y)
			max_y=polygoncorners[i].y;
		if (polygoncorners[i].x < min_x)
			min_x=polygoncorners[i].x;
		if (polygoncorners[i].y < min_y)
			min_y=polygoncorners[i].y;
	}

	for (int i=mc.x; i<max_x; i++)
	{
		for (int j=mc.y; j<max_y; j++)
		{
			double test=pointPolygonTest(polygoncorners,Point(i,j),1);
			if (test>=1)
			{
				mc_new=Point(i,j);
				break;
			}
		}
		if (test>=1)
			{
			break;
			}
	}
	mc=mc_new;

	floodFill(maskfit1, mc, 255);
	return maskfit1;
}

Mat polygonfittingmask(Mat sampleframe, std::vector<cv::Point2i> polygoncorners)
{
	//Create fitting mask from corners
	// Draw polygon
	Mat maskfit1;
	maskfit1 = Mat::zeros(sampleframe.rows+2, sampleframe.cols+2, CV_8UC1);
	for (int i=0;i<polygoncorners.size()-1;i++)
		{
		line(maskfit1,polygoncorners[i],polygoncorners[i+1], 255,1,8);
		}
	line(maskfit1,polygoncorners[polygoncorners.size()-1],polygoncorners[0], 255,1,8);

	/// Get the moments
	Moments mu;
	mu = moments( polygoncorners, false );
	///  Get the mass center:
    Point2i mc, mc_new;
	mc = Point2i( mu.m10/mu.m00 , mu.m01/mu.m00 );
	double test=pointPolygonTest(polygoncorners,mc,0);

	// Find point inside polygon

	// Find axis aligned bounding box
    int max_x,max_y,min_x,min_y;

	max_y=polygoncorners[0].y;
	min_y=polygoncorners[0].y;
	max_x=polygoncorners[0].x;
	min_x=polygoncorners[0].x;

	for (int i=0; i < polygoncorners.size(); i++)
	{
		if (polygoncorners[i].x > max_x)
			max_x=polygoncorners[i].x;
		if (polygoncorners[i].y > max_y)
			max_y=polygoncorners[i].y;
		if (polygoncorners[i].x < min_x)
			min_x=polygoncorners[i].x;
		if (polygoncorners[i].y < min_y)
			min_y=polygoncorners[i].y;
	}

	for (int i=mc.x; i<max_x; i++)
	{
		for (int j=mc.y; j<max_y; j++)
		{
			double test=pointPolygonTest(polygoncorners,Point(i,j),1);
			if (test>=1)
			{
				mc_new=Point(i,j);
				break;
			}
		}
		if (test>=1)
			{
			break;
			}
	}
	mc=mc_new;

	floodFill(maskfit1, mc, 255);
	return maskfit1;
}

bool fittingcheck(Mat mask1, Mat mask2)
{
	//Checks if all points of mask2 are within mask1
	bool fitcheck=true;
	// Fitcheck
   	for (int i=0; i< mask1.cols; i++)
	{
		for (int j=0; j<mask1.rows; j++)
		{
			Scalar colour = mask2.at<uchar>((j),(i));
			Scalar colour2 = mask1.at<uchar>((j),(i));
			if (colour.val[0]==255 && colour2.val[0]!=255)
				{
				fitcheck=false;
				}
		}
	}
   	return fitcheck;
}

Mat shape_detection(Mouse_info mi, Mat cannyframe)
{
	Mat mask;
	mask.create(cannyframe.rows+2, cannyframe.cols+2, CV_8UC1);
	mask = Mat::zeros(cannyframe.rows+2, cannyframe.cols+2, CV_8U);
	floodFill(cannyframe, mask, mi.pt1, 255, 0, Scalar(), Scalar(), 4 + (255 << 8) + FLOODFILL_MASK_ONLY);
	return mask;
}

Mat fill_mask_to_frame(Mat mask, Mat ioframe, Scalar color)
{
   	for (int i=0; i< ioframe.cols; i++)
	{
		for (int j=0; j<ioframe.rows; j++){
			if (mask.at<char>((j+1),(i+1))!=0)
			{
				ioframe.at<Vec3b>(j, i)[0] = color[0];
				ioframe.at<Vec3b>(j, i)[1] = color[1];
				ioframe.at<Vec3b>(j, i)[2] = color[2];
			}
		}
	}
	return ioframe;
}

Rect MaskBoundingBox(Mat mask)
{
    Rect output;
    std::vector<cv::Point2i> maskpoints;
    //cout << mask.rows << " " << mask.cols << endl;
   	for (int i=0; i< mask.cols-1; i++)
	{
		for (int j=0; j<mask.rows-1; j++)
		{
			if (mask.at<uchar>(Point((i+1),(j+1)))==255)
			{
				maskpoints.push_back(Point((i+1),(j+1)));
			}
		}
	}
   	output= boundingRect(maskpoints);
	return output;
}

Mat Mask_in_frame(PositionedMask maskinput, Mat sampleframe)
{
	Mat newmask;
	newmask.create(sampleframe.rows+2, sampleframe.cols+2, CV_8UC1);
	newmask = Mat::zeros(sampleframe.rows+2, sampleframe.cols+2, CV_8U);
	Rect roi=maskinput.BBox;
	maskinput.Mask.copyTo(newmask(roi));
	return newmask;

}


PositionedMask cropmaskwithBB(Mat inputmask)
{
	Rect BoundingBox;
	Mat croppedmask;
	BoundingBox = MaskBoundingBox(inputmask);
	//cout << "Bounding box" << BoundingBox << endl;
	croppedmask=inputmask(BoundingBox);
	PositionedMask detected_mask;
	detected_mask.Mask=croppedmask;
	detected_mask.BBox=BoundingBox;
	return detected_mask;
}

PositionedMask rotateMask(PositionedMask inputmask, int theta)
{
	Mat frame, frameRotated;
	Mat src=inputmask.Mask;
	Rect original_position=inputmask.BBox;
	int diagonal = (int)sqrt(src.cols*src.cols+src.rows*src.rows);
	int newWidth = diagonal+10;
	int newHeight =diagonal+10;

	int offsetX = (newWidth - src.cols) / 2;
	int offsetY = (newHeight - src.rows) / 2;
	Mat targetMat;
	targetMat = Mat::zeros(newWidth, newHeight, CV_8U);

	Point2f src_center(targetMat.cols/2.0F, targetMat.rows/2.0F);

	src.copyTo(frame);
	double radians = theta * M_PI / 180.0;
	double sin = abs(std::sin(radians));
	double cos = abs(std::cos(radians));

	frame.copyTo(targetMat.rowRange(offsetY, offsetY + frame.rows).colRange(offsetX, offsetX + frame.cols));
	Mat rot_mat = getRotationMatrix2D(src_center, theta, 1.0);
	warpAffine(targetMat, frameRotated, rot_mat, targetMat.size());
	//imshow("blubb",frameRotated);
	//waitKey();
	inputmask=cropmaskwithBB(frameRotated);
	inputmask.BBox.x=original_position.x;
	inputmask.BBox.y=original_position.y;

	return inputmask;

}

std::vector<cv::Point2i> rotatecorners(std::vector<cv::Point2i> inputcorners, int theta)
{
    const double PI(3.14159265);
	/// Get the moments
	Moments mu;
	mu = moments( inputcorners, false );
	///  Get the mass center as rotation center:
    Point2i mc;
	mc = Point2i( mu.m10/mu.m00 , mu.m01/mu.m00 );

	Point retranslated;
	std::vector<cv::Point2i> outputcorners;

	cout << inputcorners.size() << endl;

	for (int i=0; i<inputcorners.size(); i++)
	{
		Point2f translated=inputcorners[i]-mc;
		Point2f rotated=Point(0,0);

		double radians = -theta * PI / 180.0;
		double sinus = sin(radians);
		double cosinus = cos(radians);

		rotated.x=translated.x*cosinus-translated.y*sinus;
		rotated.y=translated.x*sinus+translated.y*cosinus;

		//cout << "i: " << i << " mc: "<< mc << " Input:" << inputcorners[i] << " translated: " << translated << " rotated: "<< rotated << endl;

		retranslated.x=int(rotated.x+mc.x);
		retranslated.y=int(rotated.y+mc.y);
		outputcorners.push_back(retranslated);
	}

	return outputcorners;
}


using namespace std;
using namespace cv;

struct Marker_Data {
	cv::Size BoardSize;
	double Gridsize;
	long int Board_ID;
	int short_ID;
	Point3f Position; //Position of reference corner in machine
};

struct Found_Marker {
	vector<Point2f> pointBuf;
	vector<Point2i> Position; //Position in original frame
	Marker_Data* Marker_info;
};



void fill_mask_to_frame(Mat mask, Mat* ioframe, Scalar color)
{
   	for (int i=0; i< ioframe->cols; i++)
	{
		for (int j=0; j<ioframe->rows; j++){
			if (mask.at<char>((j+1),(i+1))!=0)
			{
				ioframe->at<Vec3b>(j, i)[0] = color[0];
				ioframe->at<Vec3b>(j, i)[1] = color[1];
				ioframe->at<Vec3b>(j, i)[2] = color[2];
			}
		}
	}
}




//The modes for the program
enum { Read = 0, FindChessboard = 1, IsolateMarker = 2 , IdentifyMarker = 3,
	CalculatePosition = 4, CreateCubes = 5, PositionImage = 6, AdjustWarpMatrix = 7,
	DisplayMachine = 8};

enum { READ = 0, PROCESSING=1, DETECTION = 2, DRAWING = 3, FITTING = 4,
	DISPLAY=5, MOVE=6, MASK_CREATION=7, SHAPE_DETECTION=8, AUTOFITTING=9, ROTATE=10};


int main(int argc, char* argv[])
{

//	VideoCapture vcap;
//	const string url = "http://141.58.8.56/camera/current.jpg";
//	vcap.open(url);
//
//	if (!vcap.isOpened())
//	{
//		std::cout << "could not capture" << std::endl;
//		return 0;
//	}
    const Scalar RED(0,0,255), GREEN(0,255,0), BLUE(255,0,0), PURPLE(255,0,255);

	//metal height;
    int metal_height=0;
    int old_metal_height=0;
	int iSliderValue1=0;
    namedWindow("Main Window" , CV_WINDOW_AUTOSIZE );

	// Initialize /dev Video devices
	CameraManager *Manager= new CameraManager();
//	Manager->addCamera(0,V4L,""); //Todo fix syntax to remove empty argument
//	Manager->addCamera(1,V4L,"");
//	Manager->addCamera(3,LAN,"http://141.58.8.57/camera/current.jpg");
	Manager->addCamera(0,LAN,"http://141.58.8.56/camera/current.jpg");
	//Manager->addCamera(1,UEye,"");
	//Manager->addCamera(2,UEye,"");

    std::vector<VideoCapture> captures;
	int number_cameras=Manager->Cameras.size();
	std::vector<Mat> machineviews(number_cameras);
	std::vector<Mat> viewtomachinemtx(number_cameras);
	std::vector<Mat> maskblendtransformed(number_cameras);
	std::vector<Mat> originalframe(number_cameras);
	vector<vector<Point2f> > mincornerPointdev(number_cameras,vector<Point2f>(4));
	vector<vector<Point3f> > mincorner3Ddev(number_cameras,vector<Point3f>(4));
	vector<vector<Point3f> > mincorner3Ddevheight(number_cameras,vector<Point3f>(4));
	vector<vector<Point2f> > mincornermachine(number_cameras,vector<Point2f>(4));
	vector<vector<Point2f> > mincornercamera(number_cameras,vector<Point2f>(4));
    vector<Mat> rvec(number_cameras),tvec(number_cameras); //  bring points from the model coordinate system to the camera coordinate system
	vector<Mat> rvec2(number_cameras),tvec2(number_cameras); // Position of camera in model coordinates
	vector<Mat> cameraMatrix(number_cameras) , distCoeffs(number_cameras) , NewcameraMatrix(number_cameras);
	vector<string> machdev;
	Mat transformaddandblend;


	Size machineSize; //Size in mm
	machineSize.width=297*2;
	machineSize.height=210*2;
	double resolution=2; //pixel/mm
	Mat machine=Mat::zeros( machineSize.height*resolution, machineSize.width*resolution, CV_8UC3);




	Mat undistortframe; // frame undistorted with matrices from file




// Run loop for all cameras
	int dev=0;

	while (dev<=number_cameras)
	{

		if (dev<number_cameras)
		{
		//Get Distortion matrix and prepare outputfile;
//		int dev = Manager->Cameras[i]->DeviceNumber; //ToDo Change to iterating through given device numbers instead of forcing an ordering system
		string filenameinpre="out_camera_data_";
		string filenamesuffix=".xml";
		stringstream filenameindev;
		filenameindev << filenameinpre << dev << filenamesuffix;
		string filenamein = filenameindev.str();								//Read Matrix created by camera_calibration
		FileStorage fs(filenamein, FileStorage::READ);
		cout << filenamein << " opened for reading"<< endl;

		string filenameoutpre="transformation_matrix_";
		stringstream filenameoutdev;
		filenameoutdev << filenameoutpre << dev << filenamesuffix;
		string filenameout = filenameoutdev.str();								//Read Matrix created by camera_calibration

		fs["Camera_Matrix"] >> cameraMatrix[dev];
		fs["Distortion_Coefficients"] >> distCoeffs[dev];
		}

		//Board and machine parameters
		Size boardSize;
		boardSize.width=4;
		boardSize.height=3;
		double gridsize=40.0;

		vector<Point2f> cornerPoint(4);
		vector<Point2f> mincornerPoint(4);
		vector<Point3f> mincorner3D(4);
		//vector<Point2f> mincornermachine(4);
		//vector<Point2f> mincornercamera(4);



		Mat cleanframe,cameraframe,boardsearchframe;
	    Mat isolated_Marker;
		Mat outputframewithtext; // Just before adding msg to outputframewithtext undistortframe is copied there





		bool bSuccess=true;

		std::vector<Point2f> quad_pts; //The 4 chessboard main corners in original view
		string msg="Looking for Boards...";
		vector<Point2f> pointBuf,pointBuf2; //Chessboard corners in original and isolated view

		Mat transmtx; // Matrix used for marker isolation



		int mode = Read; //First mode to enter in main loop
	    bool display=true;

	    int number_boards_found=0;

	    vector<Point3f> pointList3D;
	    vector<Point2f> pointList2D;


	    //Chessboard markers with circles, Marker id is sum(2^(circle_position))
	    vector<Found_Marker> markers_found;
	    Found_Marker markers_found_temp;

	    vector<Marker_Data> marker_data;

	    // Todo: Read marker data from file
	    Marker_Data Marker;
	    Marker.BoardSize.width=4;
	    Marker.BoardSize.height=3;
	    Marker.Gridsize=40;
	    Marker.Board_ID=135360;
	    Marker.Position=Point3f(88,66,0);
	    Marker.short_ID=1;

	    marker_data.push_back(Marker);

	    Marker.BoardSize.width=4;
	    Marker.BoardSize.height=3;
	    Marker.Gridsize=40;
	    Marker.Board_ID=74304;
	    Marker.Position=Point3f(88+286,66,0);
	    Marker.short_ID=2;

	    marker_data.push_back(Marker);

	    Marker.BoardSize.width=4;
	    Marker.BoardSize.height=3;
	    Marker.Gridsize=40;
	    Marker.Board_ID=12864; //
	    Marker.Position=Point3f(88+286,66+187,0);
	    Marker.short_ID=3;

	    marker_data.push_back(Marker);

	    Marker.BoardSize.width=4;
	    Marker.BoardSize.height=3;
	    Marker.Gridsize=40;
	    Marker.Board_ID=582; //
	    Marker.short_ID=4;
	    Marker.Position=Point3f(88,66+187,0);
	    marker_data.push_back(Marker);

	    bool run=true;
		while (run)
		{
			switch (mode)
			{
				case Read:
				{
					//msg="Reading frame from camera";
					Manager->capture();
					for (int i=0; i<number_cameras; i++)
					{
						originalframe[i]=Manager->Cameras[i]->originalframe;
						bool cSuccess = Manager->Cameras[i]->get_success; // read a new frame from video
						bSuccess = bSuccess & cSuccess;
					}

					if (!bSuccess) //if not success, break loop
					{
						cout << "Cannot read a frame from video stream" << endl;
						break;
					}

					if (number_cameras>1)
					{
						//imshow("original 0",originalframe[0]);
						//imshow("original 1",originalframe[1]);

						for (int i = 0; i < number_cameras; i++) {
							imshow("original " + i, originalframe[i]);
						}
					}



					if (dev==number_cameras)
					{
						mode = DisplayMachine;
					}
					else
					{
						cameraframe = originalframe[dev];
						// ToDo: use undistortion matrix + sort matrices
						//undistort(cameraframe, undistortframe, cameraMatrix[dev], distCoeffs[dev]);

						undistortframe=cameraframe.clone(); //Chessboard corners and cubes are drawn here
						cleanframe = undistortframe.clone(); // Boards for marker identification are isolated from a clean view
						boardsearchframe = undistortframe.clone(); // Identified markers are erased with a pattern so findChessboardCorners can find the next one

						//Clearing results of previous search
						pointList3D.clear();
						pointList2D.clear();
						markers_found.clear();
						number_boards_found=0;
						mincornerPoint.clear();
						cornerPoint.clear();
						mincornerPoint[3]=Point2f(0.0,0.0);
						mincornerPoint[2]=Point2f(0.0,(float)undistortframe.rows);
						mincornerPoint[1]=Point2f((float)undistortframe.cols,0.0);
						mincornerPoint[0]=Point2f((float)undistortframe.cols,(float)undistortframe.rows);

						cornerPoint[0]=Point2f(0.0,0.0);
						cornerPoint[1]=Point2f(0.0,(float)undistortframe.rows);
						cornerPoint[2]=Point2f((float)undistortframe.cols,0.0);
						cornerPoint[3]=Point2f((float)undistortframe.cols,(float)undistortframe.rows);

						mode = FindChessboard;
					}
				}
				break;

				case FindChessboard:
				{
					bool found;

					found = findChessboardCorners( boardsearchframe, boardSize, pointBuf,
						 CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
					if ( found)                // If done with success,
					{
						msg=" Chessboard found ";
						number_boards_found=number_boards_found+1;
						mode= IsolateMarker;
						markers_found_temp.pointBuf=pointBuf;

						  // improve the found corners' coordinate accuracy for chessboard
							{
								Mat viewGray;
								cvtColor(undistortframe, viewGray, COLOR_BGR2GRAY);
								cornerSubPix( viewGray, pointBuf, Size(11,11),
									Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
							}
							// Draw the corners.
							drawChessboardCorners( undistortframe, boardSize, Mat(pointBuf), found );

							//cout << pointBuf[0] <<" "<< pointBuf[boardSize.width-1] << endl;



					}
			        else
			        {

			            stringstream boardsfoundtext;
			            boardsfoundtext << "Found " << number_boards_found << " boards";
			            msg=boardsfoundtext.str();
			            cout << boardsfoundtext.str()<< endl;;
			            if (number_boards_found>0)
			            {
				            mode = CalculatePosition;
							display=true;
			            }
			            else
			            {
				            mode = Read;
							display=true;
			            }

			        }
				}
				break;

				case IsolateMarker:
				{
					std::vector<Point2f> cube_pts;

					//Q1-Q4: The 4 corners in the original frame system
					Point2f Q1,Q2,Q3,Q4;

					quad_pts.clear();
					Q1=pointBuf[0];
					Q2=pointBuf[boardSize.width-1];
					Q3=pointBuf[(boardSize.width*boardSize.height)-1];
					Q4=pointBuf[(boardSize.width*boardSize.height)-(boardSize.width)];

					quad_pts.push_back(Q1);
					quad_pts.push_back(Q2);
					quad_pts.push_back(Q3);
					quad_pts.push_back(Q4);

					// compute the size of the inner board dimensions
					double markerW=((double)boardSize.width-1.0)*gridsize*resolution;
					double markerH=((double)boardSize.height-1.0)*gridsize*resolution;


					Rect R(1.5*gridsize*resolution,1.5*gridsize*resolution,markerW,markerH);
					Size isolated_Marker_Size;
					isolated_Marker_Size.height=((double)boardSize.height+2.0)*gridsize*resolution;
					isolated_Marker_Size.width=((double)boardSize.width+2.0)*gridsize*resolution;

					isolated_Marker = Mat::zeros( isolated_Marker_Size.height, isolated_Marker_Size.width, CV_8UC3);

					//R1-R4 The 4 corners in an isolated frame for marker identification
					Point R1=Point2f(R.x,R.y);
					Point R2=Point2f(R.x+R.width,R.y);
					Point R3=Point2f(R.x+R.width,R.y+R.height);
					Point R4=Point2f(R.x,R.y+R.height);

					std::vector<Point2f> squre_pts;
					squre_pts.push_back(R1);
					squre_pts.push_back(R2);
					squre_pts.push_back(R3);
					squre_pts.push_back(R4);

					transmtx = getPerspectiveTransform(quad_pts,squre_pts);

					warpPerspective(cleanframe, isolated_Marker, transmtx, isolated_Marker.size());
					mode=IdentifyMarker;
				}
				break;

				case IdentifyMarker:
				{
					//Vector of found circles in Marker
			        vector<bool> Board_circle(pointBuf.size());

					//Calculate Corners in isolated view
			        pointBuf2.clear();
			        for (int k=0; k<boardSize.height+1;k++)
			        {
					    for (int j=0; j<boardSize.width+1;j++)
				        {
				        	Point2f pointbuf_temp=gridsize*resolution*Point2f(0.5,0.5)+gridsize*resolution*Point2f(j,k);
				        	pointBuf2.push_back(pointbuf_temp);
				        	//cout << pointbuf_temp << endl;
				        }
			        }

			        long int Board_ID=0;

//			        namedWindow( "gray", CV_WINDOW_AUTOSIZE ); //debug
					//Look for circles
			        for (uint i_corner=0;i_corner<pointBuf2.size();i_corner++)
			        {
						Rect region_of_interest = Rect(pointBuf2[i_corner].x,
								pointBuf2[i_corner].y,
								gridsize*resolution,
								gridsize*resolution);

						Mat image_roi = isolated_Marker(region_of_interest);

					    Mat gray;

					    cvtColor(image_roi, gray, COLOR_BGR2GRAY);

					    medianBlur(gray, gray, 5);


//					    vector<Vec3f> circles;
//
//					    HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 10,
//					    		100, 30, 1, 70 // change the last two parameters
//								// (min_radius & max_radius) to detect larger circles
//					    );
//					    if (circles.size()>0)
//					    {

					    uchar squareColor = gray.at<uchar>(5,5);//TODO find better place for color
					    uchar centerColor = gray.at<uchar>(gray.rows/2,gray.cols/2);

					    if (!(centerColor-100<=squareColor && centerColor+100>=squareColor))
					    //the color of the center is not close to the color of the rest of the square
					    {
					    	Board_circle[i_corner]=true; //then there is a circle there
					    	Board_ID=Board_ID+pow(2,i_corner);
//				        	//cout << "i_corner " << i_corner <<" BoardID "<< Board_ID << endl;
					    }
					    else
					    {
					    	Board_circle[i_corner]=false;
					    }
				    	rectangle(isolated_Marker,region_of_interest,Board_circle[i_corner]?GREEN:RED,1,8,0);
					    //waitKey();
			        }
		        	//cout  <<" BoardID "<< Board_ID << endl;
					//imshow("Isolated Marker", isolated_Marker);


					bool marker_detected=false;

					for (uint i_marker=0; i_marker<marker_data.size(); i_marker++)
					{
				        if (Board_ID == marker_data[i_marker].Board_ID)
				        {
				        	//cout << "Marker " << i_marker+1 << " was detected" << endl;
				        	marker_detected=true;

				        	markers_found_temp.Marker_info=&marker_data[i_marker];
				        	markers_found.push_back(markers_found_temp);
				        	Marker=marker_data[i_marker];

				        	//calculate matching 3d points for PointBuf
					        for (int k=0; k<boardSize.height;k++)
					        {
							    for (int j=0; j<boardSize.width;j++)
						        {
							    	Point3f pointBuf3D_temp;
						        	pointBuf3D_temp=Point3f(
						        			marker_data[i_marker].Position.x,
						        			marker_data[i_marker].Position.y,
						        			marker_data[i_marker].Position.z)
						        			+ gridsize*Point3f(j,k,0);
						        	//cout << " Point3D_temp"<< pointBuf3D_temp << endl;


						        	pointList3D.push_back(pointBuf3D_temp);

						        	//cout << pointbuf_temp << endl;
						        }
					        }

					        // Put corresponding 2D-points into vector
					        for (uint k=0; k<pointBuf.size();k++)
							{
								pointList2D.push_back(pointBuf[k]);
					        	//cout << " Point2D_temp"<< pointBuf[k] << endl;
							}
				        	//cout << " BoardID "<< Board_ID << endl;
				        }
					}

					Mat mask = polygonfittingmask(boardsearchframe,quad_pts);
					fill_mask_to_frame(mask,&boardsearchframe,BLUE);
					//imshow("Boardsearchframe",boardsearchframe);
					if (marker_detected)
					{
						mode=FindChessboard;
					}
					else
			        {
			        	cout <<  "At least one marker could not be identified" << endl;
			        	//display=true;
			        	mode = Read;
			        }

				}
				break;

				case CalculatePosition:
				{
					bool PNP_found=solvePnP(pointList3D, pointList2D, cameraMatrix[dev], distCoeffs[dev], rvec[dev], tvec[dev], 0, SOLVEPNP_ITERATIVE );

					//cout << "rvec: "<< rvec[dev] << endl;
					//cout << "tvec: "<< tvec[dev] << endl;
					//cout << "pointList3D.size: " << pointList3D.size() << endl;

					// All that is needed is found by solvePNP, the inverted view matrix could be useful but not yet...
					/*
					Mat viewMatrix = Mat::zeros(4, 4, CV_64F);
					Mat viewMatrix_inverted = Mat::zeros(4, 4, CV_64F);
					Mat viewMatrix_inverted_truncated = Mat::zeros(3, 4, CV_64F);

					cv::Mat rotation;
					cv::Rodrigues(rvec[dev], rotation);

					for(unsigned int row=0; row<3; ++row)
					{
					   for(unsigned int col=0; col<3; ++col)
					   {
						  viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
					   }
					   viewMatrix.at<double>(row, 3) = tvec[dev].at<double>(row, 0);
					}
					viewMatrix.at<double>(3, 3) = 1.0f;

					invert(viewMatrix,viewMatrix_inverted,DECOMP_LU);

					for(unsigned int row=0; row<2; ++row)
					{
					   for(unsigned int col=0; col<3; ++col)
					   {
						   viewMatrix_inverted_truncated.at<double>(row, col) = rotation.at<double>(row, col);
					   }
					}

					//cout << "view Matrix inverted: "<< viewMatrix_inverted << endl;

					Mat rotation2 = Mat::zeros(3, 3, CV_64F);
					for(unsigned int row=0; row<3; ++row)
					{
					   for(unsigned int col=0; col<3; ++col)
					   {
						  rotation2.at<double>(row, col) = viewMatrix_inverted.at<double>(row, col);
					   }
					}

					tvec2[dev] = Mat::zeros(3, 1, CV_64F);

					for(unsigned int row=0; row<3; ++row)
					{
						  tvec2[dev].at<double>(row, 0) = viewMatrix_inverted.at<double>(row, 3);
					}
					cv::Rodrigues(rotation2, rvec2[dev]);

					//Calculate Camera Position in warped frame
					Point2i center=Point2i(tvec2[dev].at<double>(0, 0),tvec2[dev].at<double>(1, 0));
					circle(machine, resolution*center , 5, GREEN,2,8);

					cout << "rvec2: "<< rvec2[dev] << endl;
					cout << "tvec2: "<< tvec2[dev] << endl;*/

					mode=CreateCubes;
				}
				break;

				case CreateCubes:
				{
					for (uint i=0 ; i<markers_found.size();i++)
					{
						int cubegrid=markers_found[i].Marker_info->Gridsize;
						Size Cubesize=markers_found[i].Marker_info->BoardSize;
						Point3f S=markers_found[i].Marker_info->Position;

						int cube_elevation=0;
						int cubeheight=-2;
						vector<Point3f> cube(8); //Cube at Origin (-1 to have cube at outer corners)
						vector<Point2f> cube2(8);

						cube[0] = S + cubegrid * Point3f(-1,-1,cube_elevation);
						cube[1] = S + cubegrid * Point3f(Cubesize.width,-1,cube_elevation);
						cube[2] = S + cubegrid * Point3f(Cubesize.width,Cubesize.height,cube_elevation);
						cube[3] = S + cubegrid * Point3f(-1,Cubesize.height,cube_elevation);

						cube[4] = S + cubegrid * Point3f(-1,-1,cube_elevation+cubeheight);
						cube[5] = S + cubegrid * Point3f(Cubesize.width,-1,cube_elevation+cubeheight);
						cube[6] = S + cubegrid * Point3f(Cubesize.width,Cubesize.height,cube_elevation+cubeheight);
						cube[7] = S + cubegrid * Point3f(-1,Cubesize.height,cube_elevation+cubeheight);


					    projectPoints(cube, rvec[dev], tvec[dev], cameraMatrix[dev], distCoeffs[dev], cube2 );

					    	line(undistortframe,cube2[0],cube2[1], RED,2,8);
					    	line(undistortframe,cube2[1],cube2[2], RED,2,8);
					    	line(undistortframe,cube2[2],cube2[3], RED,2,8);
					    	line(undistortframe,cube2[3],cube2[0], RED,2,8);

					    	line(undistortframe,cube2[0],cube2[4], BLUE,2,8);
					    	line(undistortframe,cube2[1],cube2[5], BLUE,2,8);
					    	line(undistortframe,cube2[2],cube2[6], BLUE,2,8);
					    	line(undistortframe,cube2[3],cube2[7], BLUE,2,8);

					    	line(undistortframe,cube2[4],cube2[5], GREEN,2,8);
					    	line(undistortframe,cube2[5],cube2[6], GREEN,2,8);
					    	line(undistortframe,cube2[6],cube2[7], GREEN,2,8);
					    	line(undistortframe,cube2[7],cube2[4], GREEN,2,8);

							stringstream markername;
							markername << "Marker " << markers_found[i].Marker_info->short_ID;
							putText( undistortframe, markername.str(), (cube2[4]+cube2[6])/2, 1, 1, true ?  GREEN : RED);


						// Look for the points closest to the corners

						for (uint j=0; j<4; j++)
						{
							for (uint k=0; k<4; k++)
							{
								if (norm(cornerPoint[k]-cube2[j])<norm(cornerPoint[k]-mincornerPoint[k]))
								{
									mincornerPoint[k]=cube2[j];
									mincorner3D[k]=cube[j];
								}
							}
						}
					}
					circle(undistortframe, mincornerPoint[0] , 5, GREEN,2,8);
					circle(undistortframe, mincornerPoint[1] , 5, GREEN,2,8);
					circle(undistortframe, mincornerPoint[2] , 5, GREEN,2,8);
					circle(undistortframe, mincornerPoint[3] , 5, GREEN,2,8);

					mincornerPointdev[dev][0]=mincornerPoint[0];
					mincornerPointdev[dev][1]=mincornerPoint[1];
					mincornerPointdev[dev][2]=mincornerPoint[2];
					mincornerPointdev[dev][3]=mincornerPoint[3];

					mincorner3Ddev[dev][0]=mincorner3D[0];
					mincorner3Ddev[dev][1]=mincorner3D[1];
					mincorner3Ddev[dev][2]=mincorner3D[2];
					mincorner3Ddev[dev][3]=mincorner3D[3];




					display=true;
					mode=PositionImage;
				}
				break;


				case PositionImage:
				{

					for (uint i=0; i<4; i++)
					{
						mincornermachine[dev][i]=resolution*Point2f(mincorner3D[i].x,mincorner3D[i].y);
						mincornercamera[dev][i]=Point2f(mincornerPoint[i].x,mincornerPoint[i].y);
					}

					//create the machine view for each camera
					//cout << mincornercamera.size() << mincornermachine.size() << endl;

					viewtomachinemtx[dev] = getPerspectiveTransform(mincornercamera[dev],mincornermachine[dev]);

					warpPerspective(cleanframe, machine, viewtomachinemtx[dev], machine.size());
					//cout << "Width: " << machine.cols << ", Height: " << machine.rows << endl;

					machine.copyTo(machineviews[dev]);
					stringstream machdevtemp;
					machdevtemp << "Machine " << dev;
					machdev.push_back(machdevtemp.str());
					imshow(machdev[dev],machineviews[dev]);

					//cout << "dev: " << dev << endl;


					dev=dev+1;
					if (dev<number_cameras)
					{
						run=false;
					}
					mode = Read;

				}
				break;


			case DisplayMachine:
			{
				metal_height=iSliderValue1;
				//cout << "entering machine display mode" << endl;

				for (int i=0; i<number_cameras; i++)
				{
					warpPerspective(originalframe[i], machineviews[i], viewtomachinemtx[i], machine.size());
				}

				for (int i=0; i<number_cameras; i++)
					{
						vector<Mat> maskblend(number_cameras);

						maskblend[i].create(originalframe[i].rows, originalframe[i].cols, CV_8UC1);

						for (int k=0; k< maskblend[i].cols; k++)
						{
							for (int j=0; j<maskblend[i].rows; j++)
							{
								double value=((-4.0/double(maskblend[i].rows)/double(maskblend[i].rows)*double(j)*double(j-maskblend[i].rows)))*(-4.0/double(maskblend[i].cols)/double(maskblend[i].cols)*double(k)*double(k-maskblend[i].cols));
								value = (value<=0) ? 0 : ((value>=1) ? 1 : value);

								//cout << "Value: " << int(value*255) << endl;
								maskblend[i].at<uchar>(Point((k),(j))) = int(value*255);
							}
						}
						Mat maskblendtransformedtemp;
						//imshow("Maskblend",maskblend[i]);
						//waitKey();
						warpPerspective(maskblend[i], maskblendtransformedtemp, viewtomachinemtx[i], machine.size());
						maskblendtransformed[i]=maskblendtransformedtemp;

					}
				transformaddandblend=Mat::zeros(machine.rows,machine.cols,CV_8UC3);

				//cout << "still in machine display mode" << endl;

				//Todo: Adjust for more than 2 cameras
			   	for (int i=0; i< machineviews[0].cols; i++)
				{
					for (int j=0; j<machineviews[0].rows; j++)
					{
						for (int k=0; k<3; k++)
						{
							//cout << k << endl;
							//value: Sum of color values of a pointed weighted by distance from center (maskblend)
							//value2: Sum of weights (maskblend)
							int value=0;
							int value2=0;
							for (int l=0; l<number_cameras; l++)
							{
							   	value = value+((maskblendtransformed[l].at<uchar>(Point((i),(j))))*machineviews[l].at<Vec3b>(Point((i),(j)))[k]);
								value2= value2+(maskblendtransformed[l].at<uchar>(Point((i),(j))));

							}
							int value3=((value2!=0) ? value/(value2):0);
							transformaddandblend.at<Vec3b>(Point((i),(j)))[k] = value3;
						}
					}
				}

			   	for (int i=0;i<number_cameras;i++)
			   	{
			   		destroyWindow(machdev[i]);
			   	}

			   	imshow("Machine View",transformaddandblend);
			    createTrackbar( "metal height: ", "Machine View", &iSliderValue1, 100);



			   	if (metal_height!=old_metal_height)
			   	{
			   		mode=AdjustWarpMatrix;
			   		cout << "metal height changed" << endl;
			   	}
			   	else
			   	{
				   	mode=Read;
			   	}
			    display=false;

			}
			break;

			case AdjustWarpMatrix:
			{
				// Get metal height into mincorner3D

				cout << "using new metal height: " << metal_height << endl;

				for (int i_dev=0; i_dev<number_cameras; i_dev++)
				{
					for (uint i=0; i<4; i++)
					{
						mincorner3Ddevheight[i_dev][i].x=mincorner3Ddev[i_dev][i].x;
						mincorner3Ddevheight[i_dev][i].y=mincorner3Ddev[i_dev][i].y;
						mincorner3Ddevheight[i_dev][i].z=mincorner3Ddev[i_dev][i].z-metal_height;
						old_metal_height=metal_height;
					}

					projectPoints(mincorner3Ddevheight[i_dev], rvec[i_dev], tvec[i_dev], cameraMatrix[i_dev], distCoeffs[i_dev], mincornerPointdev[i_dev] );

					for (uint i=0; i<4; i++)
					{
						mincornermachine[i_dev][i]=resolution*Point2f(mincorner3Ddev[i_dev][i].x,mincorner3Ddev[i_dev][i].y);
						mincornercamera[i_dev][i]=Point2f(mincornerPointdev[i_dev][i].x,mincornerPointdev[i_dev][i].y);
					}

					//create the machine view for each camera
					//cout << mincornercamera.size() << mincornermachine.size() << endl;

					viewtomachinemtx[i_dev] = getPerspectiveTransform(mincornercamera[i_dev],mincornermachine[i_dev]);

					warpPerspective(originalframe[i_dev], machine, viewtomachinemtx[i_dev], machine.size());
					//cout << "Width: " << machine.cols << ", Height: " << machine.rows << endl;

					machine.copyTo(machineviews[i_dev]);
				}
				mode = Read;
			}
			break;

			}
			char savekey = waitKey(1);
			if (savekey == 32)
			{
				cout << "calibration for camera " << dev << " done" << endl;
				//FileStorage fsout(filenameout, FileStorage::WRITE);
				//fsout<<"Transformation_Matrix" << transmtx;
				dev=dev+1;
				if (dev<number_cameras)
				{
					run=false;
				}
				mode = Read;
			}
			if (savekey == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				if (dev<=number_cameras)
				{
					run=false;
				}

				dev=dev+1;
			}
			//waitKey();

			if (!bSuccess) //if not success, break loop
			{
			cout << "Cannot read a frame from video stream" << endl;
			break;
			}

			//----------------------------- Output Text ------------------------------------------------

			if (display)
			{
				int baseLine = 0;
				Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
				Point textOrigin(undistortframe.cols/2 - textSize.width/2, undistortframe.rows - 2*baseLine - 10);
				undistortframe.copyTo(outputframewithtext);
				putText( outputframewithtext, msg, textOrigin, 1, 1, true ?  GREEN : RED);
				resizeWindow("Main Window",outputframewithtext.cols,outputframewithtext.rows);
				imshow("Main Window", outputframewithtext);
				display=false;
				//waitKey();
			}
		}
    }











	//

	int iSliderValue2=100, iSliderValue3=300,iSliderValue4=70;
	int iSliderValue5=80;
	int iSliderValue6=0,iSliderValue7=0;
    cv::Point2i ptold(0,0);
    Mouse_info mi;
    Mat cannyframe, mask, maskfit1, outputframe, outputframewithtext,
    	transformedwithmetal,transformedwithmetalandshape, croppedmask;
    Mat greyframe, transformed, transformedadded;
    std::vector<cv::Point2i> corners,unrotatedcorners,rotatedcorners;
    bool fitcheck;
    string msg=" ";
    int mode = PROCESSING; //First mode to enter in main loop
    int counter=1;
    std::vector<PositionedMask> masks_to_fit;
    bool init_AutoFit=false;
	int stepsize=10, x_steps=0, y_steps=0;
	PositionedMask Autofittingmask,Autofittingmaskrotated,rotatedMask,unrotatedMask;
	int Autofittingtheta=0;
	int stepresolution=10,stepsize_x=1,stepsize_y=1,minstepsize=5;
	int rotation=0;
	int mouserotation=0;
	int manualrotation=0;
	int oldrotation=0;
    const double PI(3.14159265);
    bool manual_rot=false;

	Rect AutofittingcornersrotatedBBox;
	std::vector<cv::Point2i> Autofittingcorners,Autofittingcornersrotated;
	Point2i init_corners_pos;

   namedWindow("Perspective corrected" , CV_WINDOW_AUTOSIZE );
   createTrackbar( "lowerThreshold: ", "Perspective corrected", &iSliderValue2, 300);
   createTrackbar( "upperThreshold: ", "Perspective corrected", &iSliderValue3, 300);

    bool run=true;
    while (run)
    {


    	int lowerThreshold =iSliderValue2, upperTreshold=iSliderValue3;
    	int HoughLinesThreshold=iSliderValue4;
        int ThresholdP=iSliderValue5;
        int minLineLength=iSliderValue6, maxLineGap=iSliderValue7;
        bool bSuccess=true;

    	setMouseCallback("Perspective corrected", MouseEvent, (void*)&mi);

        switch (mode)
        {
        	case READ: //ToDo Ask is there not supposed to be a break here?
        	{

        	}
        	break;

        	case PROCESSING:
        	{

				transformaddandblend.copyTo(transformed);


				cvtColor( transformed, greyframe, COLOR_BGR2GRAY );
				Canny( greyframe, cannyframe, lowerThreshold, upperTreshold, 3 );
				mode = DETECTION;
				//mode = READ;
				ptold=Point(0,0);
				transformed.copyTo(outputframe);
				cout << "Image processed!" << endl;

        	}
    		break;

        	case DETECTION:
        	{
				msg = "Detecting metal shape - click to select, press d for drawing and s for shape detection";

				if (mi.pt1!=ptold)
				{
					mask = shape_detection(mi,cannyframe);
					ptold=mi.pt1;
					transformed.copyTo(transformedwithmetal);
	        		// Fill the detected shape
					transformedwithmetal=fill_mask_to_frame(mask,transformedwithmetal,BLUE);
	               	transformedwithmetal.copyTo(outputframe);
				}
        	}
			break;

        	case DRAWING:
        	{
            	msg = "draw a polygon and press c to create a mask and enter fitting mode";
            	if (mi.pt1!=ptold)
            	{
            		corners.push_back(mi.pt1);
            		ptold=mi.pt1;
            		cout << corners.size() << endl;
            		outputframe=drawPolygon(outputframe,corners,0,0,RED);
            	}
        	}
        	break;

        	case SHAPE_DETECTION:
        	{
            	msg = "select shape by clicking and press x to proceed ";
            	if (mi.pt1!=ptold)
				{
					maskfit1 = shape_detection(mi,cannyframe);
					ptold=mi.pt1;
					transformedwithmetal.copyTo(transformedwithmetalandshape);
					// Fill the detected shape
					transformedwithmetalandshape=fill_mask_to_frame(maskfit1,transformedwithmetalandshape,PURPLE);
					transformedwithmetalandshape.copyTo(outputframe);

	            	masks_to_fit.push_back(cropmaskwithBB(maskfit1));
	            	cout << masks_to_fit[0].BBox.x << endl;
				}

        	}
        	break;

        	case MASK_CREATION:
        	{
        		if (corners.size()>=3)
        		{
        			maskfit1 = polygonfittingmask(cannyframe, corners);
        			masks_to_fit.clear();
	            	masks_to_fit.push_back(cropmaskwithBB(maskfit1));
        		}

    			maskfit1=Mask_in_frame(masks_to_fit[masks_to_fit.size()-1],cannyframe);

        		mode = FITTING;
        	}
        	break;

        	case FITTING:
        	{
        		fitcheck= fittingcheck(mask, maskfit1);
        		mode=DISPLAY;

        	}
        	break;

        	case DISPLAY:
        	{


               	// Fill the tested shape depending on fitcheck result
               	transformedwithmetal.copyTo(outputframe);
        		outputframe = fill_mask_to_frame(maskfit1,outputframe,(fitcheck==true) ? GREEN : RED);

        		//outputframe=drawPolygon(outputframe,corners,1,0,RED);

        		mode = MOVE;
        		if (init_AutoFit==true && fitcheck==false)
        		{
        			mode = AUTOFITTING;
        		}
        		if (init_AutoFit==true && fitcheck==true)
        		{
        			init_AutoFit=false;
        		}

			}
        	break;



        	case MOVE:
        	{
        		if (fitcheck)
        		{
                	msg = "Fitting successful, you can still grab and move object";

        		}
        		else
        		{
                	msg = "Fitting unsuccessful, grab and move object";

        		}
            	// Check if click inside object

        		//move object
            	if (mi.pt2.x!=0 || mi.pt2.y!=0)
            	{

            		for (int i=0; i < corners.size(); i++)
            		{
            			corners[i]=corners[i]+mi.pt2;
            		}
            		if (rotation!=0)
            		{
                		for (int i=0; i < unrotatedcorners.size(); i++)
                		{
                			unrotatedcorners[i]=unrotatedcorners[i]+mi.pt2;
                		}
            		}

            		if (masks_to_fit.size()>=1)
            		{
            			masks_to_fit[masks_to_fit.size()-1].BBox.x=masks_to_fit[masks_to_fit.size()-1].BBox.x+mi.pt2.x;
            			masks_to_fit[masks_to_fit.size()-1].BBox.y=masks_to_fit[masks_to_fit.size()-1].BBox.y+mi.pt2.y;
            		}
                	mode = MASK_CREATION;
            	}
            	mi.pt2=cv::Point(0,0);


            	if (mi.pt3.x!=0 || mi.pt3.y!=0)
            	{
        			if (mi.RIGHTBUTTONUP)
        			{
        				oldrotation=oldrotation-mouserotation;
        				mi.RIGHTBUTTONUP=false;
        			}

            		Point2f rot_center;

            		if (corners.size()>=3)
            		{
            			Moments mu;
            			mu = moments( corners, false );
            			rot_center = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
            		}
            		else if (masks_to_fit.size()>=1)
            		{
            			PositionedMask rot=masks_to_fit[masks_to_fit.size()-1];
            			rot_center=Point2f(rot.BBox.x+rot.BBox.width/2.0F,rot.BBox.y+rot.BBox.height/2.0F);
            		}

           			double startangle=atan2((mi.pt1.x-rot_center.x),(mi.pt1.y-rot_center.y))*180/PI;
					double currentangle=atan2((mi.pt3.x-rot_center.x),(mi.pt3.y-rot_center.y))*180/PI;
					double rotationangle=startangle-currentangle;
					//cout << startangle << ", " << rotationangle << endl;
					mouserotation=(int)rotationangle;

                	mode = ROTATE;
            	}

            	mi.pt3=cv::Point(0,0);
        	}
        	break;

        	case AUTOFITTING:
        	{

        		PositionedMask Metalshape;
        		Metalshape.BBox=MaskBoundingBox(mask);
        		Metalshape.Mask=mask;


        		if (corners.size()>=3)
        		{
            		if (fitcheck==false && init_AutoFit==false)
            		{
            			Autofittingcorners=corners;
            			Autofittingcornersrotated=corners;
            			cout << "Autofitting for corners activated, setting switch to 1"<< endl;

            			AutofittingcornersrotatedBBox.x = Metalshape.BBox.x;
            			AutofittingcornersrotatedBBox.y = Metalshape.BBox.y;




            			init_AutoFit=true;
            		}

            		if (fitcheck==false)
            		{
            		    int max_x,max_y,min_x,min_y;

            			max_y=Autofittingcornersrotated[0].y;
            			min_y=Autofittingcornersrotated[0].y;
            			max_x=Autofittingcornersrotated[0].x;
            			min_x=Autofittingcornersrotated[0].x;

            			for (int i=0; i < Autofittingcornersrotated.size(); i++)
            			{
            				if (Autofittingcornersrotated[i].x > max_x)
            					max_x=Autofittingcornersrotated[i].x;
            				if (Autofittingcornersrotated[i].y > max_y)
            					max_y=Autofittingcornersrotated[i].y;
            				if (Autofittingcornersrotated[i].x < min_x)
            					min_x=Autofittingcornersrotated[i].x;
            				if (Autofittingcornersrotated[i].y < min_y)
            					min_y=Autofittingcornersrotated[i].y;
            			}
            			AutofittingcornersrotatedBBox.width = max_x-min_x;
            			AutofittingcornersrotatedBBox.height = max_y-min_y;
            			init_corners_pos.x=min_x;
            			init_corners_pos.y=min_y;

        			stepsize_x=(Metalshape.BBox.width-AutofittingcornersrotatedBBox.width)/stepresolution;
        			stepsize_x=(stepsize_x>=minstepsize) ? stepsize_x : minstepsize;
        			stepsize_y=(Metalshape.BBox.height-AutofittingcornersrotatedBBox.height)/stepresolution;
        			stepsize_y=(stepsize_y>=minstepsize) ? stepsize_y : minstepsize;

           			AutofittingcornersrotatedBBox.x = Metalshape.BBox.x+(stepsize_x*x_steps);
           			AutofittingcornersrotatedBBox.y = Metalshape.BBox.y+(stepsize_y*y_steps);

          			if (((AutofittingcornersrotatedBBox.x
							+AutofittingcornersrotatedBBox.width)
							<= (Metalshape.BBox.x+Metalshape.BBox.width)) &&
							((AutofittingcornersrotatedBBox.y
							+AutofittingcornersrotatedBBox.height)
							<= (Metalshape.BBox.y+Metalshape.BBox.height)))
					{
						x_steps=x_steps+1;

					}
					else if ((AutofittingcornersrotatedBBox.y
							+AutofittingcornersrotatedBBox.height)
							<= (Metalshape.BBox.y+Metalshape.BBox.height))
					{
						y_steps=y_steps+1;
						x_steps=0;
					}
					else
					{
						x_steps=0; y_steps=0;
						cout <<  "auto-fittting not successful, rotate by 10deg and start again" << endl;
						Autofittingtheta+=10;
						Autofittingcornersrotated=(rotatecorners(Autofittingcorners,Autofittingtheta));
	        			AutofittingcornersrotatedBBox.x = Metalshape.BBox.x;
	        			AutofittingcornersrotatedBBox.y = Metalshape.BBox.y;

	            		if (Autofittingtheta>=360)
	            		{
	            			init_AutoFit=false;
	            			cout << "Turned Object by " << Autofittingtheta << "deg without successful fitting" << endl;
	            		}
					}
          			mi.pt2=Point(AutofittingcornersrotatedBBox.x-init_corners_pos.x,AutofittingcornersrotatedBBox.y-init_corners_pos.y);
          			corners=Autofittingcornersrotated;
          			mode = MOVE;
            		}
        		}

        		else if (masks_to_fit.size()>=1)
        		{
            		// Move selected shape to origin
            		if (fitcheck==false && init_AutoFit==false)
            		{
            			Autofittingmask=masks_to_fit[masks_to_fit.size()-1];
            			Autofittingmaskrotated=Autofittingmask;
            			cout << "Autofitting activated, setting switch to 1"<< endl;

            			Autofittingmaskrotated.BBox.x = Metalshape.BBox.x;
            			Autofittingmaskrotated.BBox.y = Metalshape.BBox.y;
            			init_AutoFit=true;


            		}

            		if (fitcheck==false)
            		{
            			stepsize_x=(Metalshape.BBox.width-Autofittingmaskrotated.BBox.width)/stepresolution;
            			stepsize_x=(stepsize_x>=minstepsize) ? stepsize_x : minstepsize;
            			stepsize_y=(Metalshape.BBox.height-Autofittingmaskrotated.BBox.height)/stepresolution;
            			stepsize_y=(stepsize_y>=minstepsize) ? stepsize_y : minstepsize;

            			//mi.pt2.x=stepsize_x*x_steps;
            			//mi.pt2.y=stepsize_y*y_steps;
               			Autofittingmaskrotated.BBox.x = Metalshape.BBox.x+(stepsize_x*x_steps);
               			Autofittingmaskrotated.BBox.y = Metalshape.BBox.y+(stepsize_y*y_steps);

               			if (((Autofittingmaskrotated.BBox.x
    							+Autofittingmaskrotated.BBox.width)
    							<= (Metalshape.BBox.x+Metalshape.BBox.width)) &&
    							((Autofittingmaskrotated.BBox.y
    							+Autofittingmaskrotated.BBox.height)
    							<= (Metalshape.BBox.y+Metalshape.BBox.height)))
    					{
    						x_steps=x_steps+1;

    					}
    					else if ((Autofittingmaskrotated.BBox.y
    							+Autofittingmaskrotated.BBox.height)
    							<= (Metalshape.BBox.y+Metalshape.BBox.height))
    					{
    						y_steps=y_steps+1;
    						x_steps=0;
    					}
    					else
    					{
    						x_steps=0; y_steps=0;
    						cout <<  "auto-fittting not successful, rotate by 10deg and start again" << endl;
    						Autofittingtheta+=10;
    						Autofittingmaskrotated=(rotateMask(Autofittingmask,Autofittingtheta));
    	        			Autofittingmaskrotated.BBox.x = Metalshape.BBox.x;
    	        			Autofittingmaskrotated.BBox.y = Metalshape.BBox.y;

    	            		if (Autofittingtheta>=360)
    	            		{
    	            			init_AutoFit=false;
    	            			cout << "Turned Object by " << Autofittingtheta << "deg without successful fitting" << endl;
    	            		}    					}
            		}
            		masks_to_fit[masks_to_fit.size()-1]=Autofittingmaskrotated;
            		mode=MASK_CREATION;

        		}

        	}
        	break;

        	case ROTATE:
        	{
        		// Unrotated object for corners and masks
        		if (corners.size()>=3)
        		{
            		if (rotation==0)
            		{
                		unrotatedcorners=corners;
            		}
        		}
        		else if (masks_to_fit.size()>=1)
        		{
            		unrotatedMask.BBox.x=masks_to_fit[masks_to_fit.size()-1].BBox.x;
            		unrotatedMask.BBox.y=masks_to_fit[masks_to_fit.size()-1].BBox.y;
            		if (rotation==0)
            		{
                		unrotatedMask=masks_to_fit[masks_to_fit.size()-1];
            		}
        		}

        		//Gather rotation angles
    			if (manual_rot)
    			{
        			manualrotation=manualrotation+10;
        			manual_rot=false;
    			}
				rotation=oldrotation+manualrotation-mouserotation;

				//Perform Rotation
        		if (corners.size()>=3)
        		{
        			rotatedcorners=rotatecorners(unrotatedcorners, rotation);
        			corners=rotatedcorners;
        		}
        		else if (masks_to_fit.size()>=1)
        		{
        			rotatedMask=rotateMask(unrotatedMask, rotation);
        			masks_to_fit[masks_to_fit.size()-1]=rotatedMask;
        		}
    			mode=MASK_CREATION;
        	}


        }


        if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}





        //----------------------------- Output Text ------------------------------------------------
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(outputframe.cols/2 - textSize.width/2, outputframe.rows - 2*baseLine - 10);
        outputframe.copyTo(outputframewithtext);
        putText( outputframewithtext, msg, textOrigin, 1, 1, fitcheck ?  GREEN : RED);



        imshow("Perspective corrected", outputframewithtext);



        char key = waitKey(1);
        if (key == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
        	cout << "esc key is pressed by user" << endl;
        	run=false;
        }
        if( key == 'd' )
        {
            mode = DRAWING;
        	cout << "entering drawing mode" << endl;
        }
        if( key == 'm' )
        {
            mode = MOVE;
        	cout << "entering move mode" << endl;
        }
        if( key == 'c' )
        {
            mode = MASK_CREATION;
        	cout << "creating a mask" << endl;
        }
        if( key == 'f' )
        {
            mode = FITTING;
        	cout << "entering fitting mode" << endl;
        }
        if( key == 'r' )
        {
            mode = READ;
        	cout << "reading new frame" << endl;
        }
        if( key == 'p' )
        {
            mode = PROCESSING;
        	cout << "entering processing mode" << endl;
        }
        if( key == 's' )
        {
            mode = SHAPE_DETECTION;
        	cout << "entering shape detection mode" << endl;
        }
        if( key == 'a' )
        {
            mode = AUTOFITTING;
        	cout << "entering auto-fitting mode" << endl;
        	x_steps=0;
        	y_steps=0;
        }
        if( key == 't' )
        {
            mode = ROTATE;
        	cout << "rotate mask" << endl;
        	manual_rot=true;
        }



    }

    return 0;
}


