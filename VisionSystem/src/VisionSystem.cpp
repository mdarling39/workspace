//============================================================================
// Name        : VisionSystem.cpp
// Author      : Michael Darling
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================



#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Global.h"
#include "Threshold.h"
#include "PnPObj.h"
#include "FPSCounter.h"

#define POSE_ERR_TOL 0.05			// Definitely acceptable, stop immediately
#define SECONDARY_ERR_TOL 0.10		// Try re-ordering LED's and pick the lowest error that
// still satisfies this requirement

//#REMOVE
/*
int readPoints(const char* pointsFilename, std::vector<cv::Point2f> &points) {
	std::ifstream pointsFile;
	pointsFile.open(pointsFilename);
	if (!pointsFile.is_open()) {
		std::cout << "Could not open points file" << std::endl;
		pointsFile.close();
		exit(-1);
	} else {
		float p1, p2;
		while (pointsFile >> p1 >> p2) {
			points.push_back(cv::Point2f(p1,p2));
		}
	}
	//std::cout << points << std::endl;
	pointsFile.close();

	return 1;
}
const char* imagePointsFilename = "/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/imagePts3RANDOM"
		".txt";
 */
//#END REMOVE




const std::string camDataFilename = "/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/PS3Eye_out_camera_data.yml";
const char* modelPointsFilename = "/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/LEDPos copy2.txt";


int main() {


#if SCREEN_AVAILABLE
	//cv::namedWindow("Original");
	cv::namedWindow("FoundBlobs");
	cv::namedWindow("PoseEstimate");
#endif /* SCREEN_AVAILABLE */

	//# REMOVE  (REPLACE THIS WITH A CAP >> FRAME CALL INSIDE OF A LOOP)
	/*
	// Add multiple images to get 5 LEDs in the same frame
	cv::Mat frame = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib1.jpg");
	cv::Mat img2 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib2.jpg");
	cv::Mat img3 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib3.jpg");
	cv::Mat img4 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib4.jpg");
	cv::Mat img5 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib5.jpg");
	cv::Mat img6 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib6.jpg");
	frame = frame + img2 + img3 + img4 + img5 + img6;

	frame = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Calibration/Zoomed In/Test Images/Test3.jpg");
	 */
	//# END REMOVE
	cv::Mat frame;
	cv::VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,640.0);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,480.0);
	if (!cap.isOpened()) {
		return -1;
	}
	for(int i=0; i<20; i++) {
		cap >> frame;
	}


	// set blob parameters
	cv::SimpleBlobDetector::Params blobParams;
	blobParams.minThreshold = 200;
	blobParams.maxThreshold = 255;
	blobParams.thresholdStep = 5;
	blobParams.minDistBetweenBlobs = 8;
	blobParams.minArea = 3;
	blobParams.maxArea = 75;
	blobParams.minCircularity = 0.3;
	blobParams.maxCircularity = 5.0;
	blobParams.blobColor = 255;  // I think this is red color (or is it "bright" pixels?)
	blobParams.filterByArea = true;
	blobParams.filterByColor = true;
	blobParams.filterByCircularity = true;
	blobParams.filterByInertia = false;
	blobParams.filterByConvexity = false;


	// initialize threshold object and set parameters
	Threshold myThreshObj;
	myThreshObj.set_params(blobParams);

	// initialize PnP object and parameters
	PnPObj myPnP;
	myPnP.setCamProps(camDataFilename);
	myPnP.setModelPoints(modelPointsFilename);


	// create fps clock object
	FPSCounter FPSclk(15);

	//# LOOP
	for (; ;) {

		//# capture frame from camera
		cap >> frame;

		//# compute fps moving average
		double fpsMAVG = FPSclk.fps();

		// Find blobs
		myThreshObj.set_image(frame);
		myThreshObj.detect_blobs();
		std::vector<cv::Point2f> imagePoints = myThreshObj.get_points();
		if (imagePoints.size() != NO_LEDS) {
			std::cout << "Number of blobs found (" << imagePoints.size() << ") not equal to number of LEDs -- breaking loop"
					<< std::endl;
#if SCREEN_AVAILABLE
			cv::imshow("PoseEstimate",frame);
			cv::waitKey(1);
#endif /* SCREEN_AVAILABLE */

			continue;	//# Don't try and solve PnP-- Grab another frame and start over
		}

#if DEBUG_MODE
		std::cout << "\n" << imagePoints.size() << " image points detected:" << std::endl;
		std::cout << imagePoints << "\n" << std::endl;
		std::cout << "Average frame rate:  " << fpsMAVG << "\n" << std::endl;
#endif /* DEBUG_MODE */



		// load image points into PNPobject
		//#REMOVE
		/*
		std::vector<cv::Point2f> fakeImagePoints;
		imagePoints = fakeImagePoints;
		readPoints(imagePointsFilename, imagePoints);
		 */
		//#END REMOVE
		myPnP.setImagePoints(imagePoints);

		// repeat pose estimate until error tolerance is met, otherwise start over with a new frame

		std::vector<double> poseState;
		double poseErr = INFINITY;

		for (int i=0; i<=2; i++) {
			myPnP.correlatePoints(i+1);  //use the next scheme (CallNo = 1-3)
			myPnP.solve();
			double thisErr = myPnP.getScaledReprojError();

			if (thisErr < POSE_ERR_TOL) { // Meets primary tolerance -- return with this state immediately
				poseErr = thisErr;
				poseState = myPnP.getState();
				break;

			} else if (thisErr < SECONDARY_ERR_TOL) { // Meets secondary tolerance -- keep,
													  // but see if we can do better

				if (thisErr < poseErr) { // Is better than the last case -- grab this state.
					poseErr = thisErr;
					poseState = myPnP.getState();
				}

			} else if (i<2) {  // This one didn't work, have to try again
				myPnP.resetGuess();
			}
		}



		//# UNCOMMENT WHEN IN LOOP
		if (poseErr > SECONDARY_ERR_TOL) {
			std::cout << "Pose estimate does not meet reprojection error tolerance -- grabbing new frame"
					<< std::endl;
#if SCREEN_AVAILABLE
			cv::imshow("PoseEstimate",frame);
			cv::waitKey(1);
#endif /* SCREEN_AVAILABLE */
			continue;
		}
		//# END UNCOMMENT WHEN IN LOOP

		// convert state from mm/rads to inches/degrees
		poseState[0] *= MM2IN;
		poseState[1] *= MM2IN;
		poseState[2] *= MM2IN;
		poseState[3] *= RAD2DEG;
		poseState[4] *= RAD2DEG;
		poseState[5] *= RAD2DEG;



#if DEBUG_MODE
		// Print state to standard output
		std::cout << "Estimated Pose:\n" << "[";
		std::copy(poseState.begin(), poseState.end()-1, std::ostream_iterator<double>(std::cout, ", "));
		std::cout << (poseState.back()) <<  "]"<< std::endl;
#endif /* DEBUG_MODE */



#if SCREEN_AVAILABLE
		// get "blobs" image
		cv::Mat blobs(frame.size(),CV_8U,cv::Scalar(0));
		myThreshObj.createBlobsImage(blobs);

		//cv::imshow("Original",frame);

		cv::imshow("FoundBlobs",blobs);

		myPnP.drawOverFrame(frame,frame);
		cv::imshow("PoseEstimate",frame);
		cv::waitKey(1);
#endif /* SCREEN_AVAILABLE */

#if HOLD_WINDOWS
		cv::waitKey(0);
#endif /* HOLD_WINDOWS */
	}

	return 0;
}
