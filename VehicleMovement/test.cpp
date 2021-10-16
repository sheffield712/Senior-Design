#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate;
Mat ROILane;
int LeftLanePos, RightLanePos, MiddlePos, frameCenter, laneCenter, Result;
int firstpin, secondpin, thirdpin, fourthpin;

// access camera
RaspiCam_Cv Camera;

stringstream ss;

// dynamic array
vector<int> histogramLane;

// create a frame of reference... adjust these as needed. They represent the 4 corners of the box.
Point2f Source[] = {Point2f(25, 120), Point2f(330, 120), Point2f(0, 210), Point2f(360, 210)};
// Point2f Destination[] = {Point2f(25, 150), Point2f(330, 150), Point2f(0, 210), Point2f(360, 210)};
Point2f Destination[] = {Point2f(80,0), Point2f(280, 0), Point2f(80, 240), Point2f(280, 240)};

void Perspective()
{
	// this is to join the 4 points via openCV
	int lineWidth = 2;
	
	line(frame, Source[0], Source[1], Scalar(0, 0, 255), lineWidth); // goes from top left to top right
	line(frame, Source[1], Source[3], Scalar(0, 0, 255), lineWidth); // goes from top right to bottom right
	line(frame, Source[3], Source[2], Scalar(0, 0, 255), lineWidth); // goes from bottom right to bottom left
	line(frame, Source[2], Source[0], Scalar(0, 0, 255), lineWidth); // goes from bottom left to top left
	
	
	Matrix = getPerspectiveTransform(Source, Destination);
	warpPerspective(frame, framePers, Matrix, Size(360, 240));
}

// make the image grayscale and up the contrast
void Threshold()
{
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
	// frame input name, min threshold for white, max threshold for white, frame output name. Tweak these as necessary, but min threshold may want to go down if indoors.
	// find the white in the image.
	inRange(frameGray, 220, 255, frameThresh); // 137 looked good indoors at night, 165 looked good indoors during the day
	// input, output, minimum threshold for histerisis process. always goes 100 for minimum. 2nd threshhold, usually go 500, axa matrix, advanced gradient?
	// edge detection
	Canny(frameGray, frameEdge, 400, 600, 3, false); // was 250, 600
	// merge our images together into final frame
	add(frameThresh, frameThresh, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	// used in histogram function only.
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);
}

void Histogram()
{
	// resize to the size of the lane.
	histogramLane.resize(frame.size().width);
	histogramLane.clear();
	
	for (int i = 0; i < frame.size().width; i++)
	{
		// reason of interest strip
		ROILane = frameFinalDuplicate(Rect(i, 140, 1, 100));
		divide(255, ROILane, ROILane);
		histogramLane.push_back((int)(sum(ROILane)[0]));
	}
}

void LaneCenter()
{
	frameCenter = 180;
	
	line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255,0,0), 3);
	
	// difference between true center and center ball...
	Result = MiddlePos - frameCenter;
}

void LaneFinder()
{
	// iterator to point to max intensity spot
	vector<int>:: iterator LeftPtr;
	// scans from left-most pixel to middle pixel
	LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() + 200);
	LeftLanePos = distance(histogramLane.begin(), LeftPtr);
	
	// iterator to point to max intensity spot
	vector<int>:: iterator RightPtr;
	// scans from left-most pixel to middle pixel
	RightPtr = max_element(histogramLane.begin() + 200, histogramLane.end());
	RightLanePos = distance(histogramLane.begin(), RightPtr);
	
	line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 250, 0), 2);
	line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0, 250, 0), 2);
}

void BallFinder()
{
	// iterator to point to max intensity spot
	vector<int>:: iterator LeftPtr;
	// scans from left-most pixel to middle pixel
	LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() + 120);
	LeftLanePos = distance(histogramLane.begin(), LeftPtr);
	
	// iterator to point to max intensity spot
	vector<int>:: iterator RightPtr;
	// scans from left-most pixel to middle pixel
	RightPtr = max_element(histogramLane.end() - 119, histogramLane.end());
	RightLanePos = distance(histogramLane.begin(), RightPtr);
	
	
	vector<int>:: iterator MiddlePtr;
	MiddlePtr = max_element(histogramLane.begin() + 121, histogramLane.end() - 120);
	MiddlePos = distance(histogramLane.begin(), MiddlePtr);
	
	// middle is at pixel column 180
	int midDist = abs(180 - MiddlePos);
	int leftDist = abs(180 - LeftLanePos);
	int rightDist = abs(180 - RightLanePos);
	
	if (midDist <= leftDist && midDist <= rightDist) {
		MiddlePos = MiddlePos;
	} else if (leftDist <= midDist && leftDist <= rightDist) {
		MiddlePos = LeftLanePos;
	} else {
		MiddlePos = RightLanePos;
	}
	
	line(frameFinal, Point2f(MiddlePos, 0), Point2f(MiddlePos, 240), Scalar(0, 250, 0), 2);
	
}

void Setup ( int argc, char **argv, RaspiCam_Cv &Camera)
{
	// Camera settings. Adjust as needed.
	// frame width set at 360 pixels
	Camera.set( CAP_PROP_FRAME_WIDTH, ( "-w", argc, argv, 360 ) );
	// frame heigh set at 240 pixels
	Camera.set( CAP_PROP_FRAME_HEIGHT, ( "-h", argc, argv, 240 ) );
	// brightness set at 50 / 100 as a default.
	Camera.set( CAP_PROP_BRIGHTNESS, ( "-br", argc, argv, 50 ) );
	Camera.set( CAP_PROP_CONTRAST, ( "-co", argc, argv, 50) );
	Camera.set( CAP_PROP_SATURATION, ( "-sa", argc, argv, 50) );
	Camera.set( CAP_PROP_GAIN, ( "-g", argc, argv, 50) );
	// FPS set at 0 means the camera will try to capture as many frames as it can.
	Camera.set( CAP_PROP_FPS, ( "-fps", argc, argv, 0) );
}

void Capture()
{
	Camera.grab();
	Camera.retrieve(frame);
	
	// convert to rgb format... input name, output name, Color conversion.
	// cvtColor(frame, frame, COLOR_BGR2RGB);
}

int main(int argc, char **argv)
{
	firstpin = 21;
	secondpin = 22;
	thirdpin = 23;
	fourthpin = 24;
	// set up pins to arduino
	wiringPiSetup();
	pinMode(firstpin, OUTPUT);
	pinMode(secondpin, OUTPUT);
	pinMode(thirdpin, OUTPUT);
	pinMode(fourthpin, OUTPUT);
	
	
	Setup(argc, argv, Camera);
	
	// useful stdout information
	cout<<"Connecting to Camera"<<endl;
	
	// returns 1 if camera opens correctly
	if (!Camera.open())
	{
		cout<<"Failed to connect"<<endl;
		return -1;
	}
	
	cout<<"Camera ID = "<<Camera.getId()<<endl;
	
	while(1) {
		auto start = std::chrono::system_clock::now();
		
		Capture();
		Perspective();
		Threshold();
		Histogram();
		BallFinder();
		LaneCenter();
		
		if (Result == 0 || Result == -59)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 0
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Forward"<<endl;
    }
    
        
    else if (Result >0 && Result <10)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 1
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right1"<<endl;
    }
    
        else if (Result >=10 && Result <20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 2
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right2"<<endl;
    }
    
        else if (Result >20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 3
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right3"<<endl;
    }
    
        else if (Result <0 && Result >-10)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 4
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left1"<<endl;
    }
    
        else if (Result <=-10 && Result >-20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 5
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left2"<<endl;
    }
    
        else if (Result <-20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 6
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left3"<<endl;
    }
		
		ss.str(" ");
		ss.clear();
		ss<<"Result = "<<Result;
		putText(frame, ss.str(), Point2f(1,50), 0, 1, Scalar(0,0,255), 2);
		
		namedWindow("original", WINDOW_KEEPRATIO);
		moveWindow("original", 0, 100);
		resizeWindow("original", 640, 480);
		imshow("original", frame);
		
		namedWindow("Perspective", WINDOW_KEEPRATIO);
		moveWindow("Perspective", 640, 100);
		resizeWindow("Perspective", 640, 480);
		imshow("Perspective", framePers);
		
		// create window for the grayscale image.
		namedWindow("GRAY", WINDOW_KEEPRATIO);
		moveWindow("GRAY", 1280, 100);
		resizeWindow("GRAY", 640, 480);
		imshow("GRAY", frameFinal);
		
		
		// wait 1 millisecond
		waitKey(1);

		auto end = std::chrono::system_clock::now();
		std:chrono::duration<double> elapsed_seconds = end - start;
		
		float t = elapsed_seconds.count();
		int FPS = 1 / t;
		// cout<<"FPS = "<<FPS<<endl;
	}
	
	return 0;
	
}
