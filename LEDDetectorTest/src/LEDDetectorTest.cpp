//============================================================================
// Name        : LEDDetectorTest.cpp
// Author      : Michael Darling
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "Threshold.h"
#include "KalmanFilter.h"
#include "FPSCounter.h"

#define FOUNDBLOBS_TO_FILE


FILE *pFile;


int main() {

	// initialize camera and capture some frames
	cv::Mat frame;
	cv::VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,640.0);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,480.0);
	cap.set(CV_CAP_PROP_FPS,60);
	if (!cap.isOpened()) {
		return -1;
	}
	for(int i=0; i<20; i++) {
		cap >> frame;
	}


	// set blob parameters
	CustomBlobDetector::Params blobParams;
	//blobParams.minRepeatability = 1;
	blobParams.maxError = 0.30;
	blobParams.maxPoints = 5;
	blobParams.minThreshold = 250;
	//blobParams.maxThreshold = 256;
	//blobParams.thresholdStep = blobParams.maxThreshold - blobParams.minThreshold - 1;
	//blobParams.minDistBetweenBlobs = 8;
	blobParams.minArea = 4;
	blobParams.maxArea = 75;
	blobParams.minCircularity = 0.4;
	blobParams.maxCircularity = 1.1;
	blobParams.targetCircularity = 1.0;
	blobParams.minInertiaRatio = 0.20;
	blobParams.maxInertiaRatio = 1.1;
	blobParams.targetInertiaRatio = 1.0;
	blobParams.minConvexity = 0.6;
	blobParams.maxConvexity = 1.1;
	blobParams.targetConvexity = 1.0;
	blobParams.targetBlobColor = 255;  // I think this is red color (or is it "bright" pixels?)

	blobParams.filterByError = false;
	blobParams.filterByArea = true;
	blobParams.filterByColor = true;
	blobParams.filterByCircularity = true;
	blobParams.filterByInertia = true;
	blobParams.filterByConvexity = true;

	blobParams.w_Circularity = 100;
	blobParams.w_InertiaRatio = 255;
	blobParams.w_Convexity = 100;
	blobParams.w_BlobColor = 50;


	// create a threshold object
	Threshold myThreshObj;
	myThreshObj.set_params(blobParams);



	// Initialize Kalman Filter
	kFilterSt kFilter;  // create a structure to hold some properties
	kFilter.n_states = 4;
	kFilter.n_measurement = 2;
	kFilter.n_controls = 0;
	kFilter.type = CV_32F;

	// create object  (most parameters are automatically allocated by constructor)
	cv::KalmanFilter KF(kFilter.n_states,kFilter.n_measurement,kFilter.n_controls,kFilter.type);

	// initialize measurement matrix (H)
	KF.measurementMatrix = (cv::Mat_<float>(kFilter.n_measurement,kFilter.n_states) <<  1, 0, 0, 0, 0, 1, 0, 0);

	// initialize covariance of process noise (Q)
	KF.processNoiseCov = cv::Mat::eye(kFilter.n_states,kFilter.n_states,kFilter.type)*(1e-2);

	// initialize covariance of measurement noise (R)
	KF.measurementNoiseCov = cv::Mat::eye(kFilter.n_measurement,kFilter.n_measurement,kFilter.type)*(1e-6);

	// initialize posterior error covariance to identity
	KF.errorCovPost = cv::Mat::eye(kFilter.n_states,kFilter.n_states,kFilter.type);

	// create variable to hold measurement
	cv::Mat z_k(kFilter.n_measurement,1,kFilter.type);


	std::cout << "statePre (x') \n" <<  KF.statePre << std::endl;
	std::cout << "statePost (x) \n" << KF.statePost << std::endl;
	std::cout << "transitionMatrix (F) \n" << KF.transitionMatrix << std::endl;
	std::cout << "controlMatrix (B) \n" << KF.controlMatrix << std::endl;
	std::cout << "measurementMatrix (H) \n" << KF.measurementMatrix << std::endl;
	std::cout << "processNoiseCov (Q) \n" << KF.processNoiseCov << std::endl;
	std::cout << "measurementNoiseCov (R) \n" << KF.measurementNoiseCov << std::endl;
	std::cout << "errorCovPre (P') \n" << KF.errorCovPre << std::endl;
	std::cout << "gain (K) \n" << KF.gain << std::endl;
	std::cout << "errorCovPost (P) \n" << KF.errorCovPost << std::endl;

	// initialize the fps clock (15 frame moving average by default)
	FPSCounter FPSclk(15);

#ifdef FOUNDBLOBS_TO_FILE
	// open file for printing to
	pFile = fopen("testfile.txt","w");

	fprintf(pFile,"%15s%15s%15s%15s%15s%15s%15s\n","weights:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15s%15d%15d%15d%15d%15s\n","(0-255)","",blobParams.w_Circularity,blobParams.w_InertiaRatio,
			blobParams.w_Convexity,blobParams.w_BlobColor,"--");

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","FilterBy:","error","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15d%15d%15d%15d%15d%15d\n","",blobParams.filterByError,blobParams.filterByCircularity,blobParams.filterByInertia,
			blobParams.filterByConvexity,blobParams.filterByColor,blobParams.filterByArea);

	fprintf(pFile,"\n%15s%15d%15.3f%15.3f%15.3f%15.3f%15.3f\n","min",0,blobParams.minCircularity,blobParams.minInertiaRatio,
			blobParams.minConvexity,blobParams.minThreshold,blobParams.minArea);
	fprintf(pFile,"%15s%15.3f%15.3f%15.3f%15.3f%15.3f%15.3f\n","max",blobParams.maxError,blobParams.maxCircularity,blobParams.maxInertiaRatio,
			blobParams.maxConvexity,255.0,blobParams.maxArea);

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","Targets:","","circularity","inertiaRatio","convexity","blobColor","area");
	fprintf(pFile,"%15s%15s%15.3f%15.3f%15.3f%15d%15s\n","","",blobParams.targetCircularity,blobParams.targetInertiaRatio,
			blobParams.targetConvexity,blobParams.targetBlobColor,"--");

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s\n","blobNo","TotalError","circularity","inertiaRatio","convexity","blobColor","area");
	fclose(pFile);
#endif  //FOUNDBLOBS_TO_FILE

	for (; ;) {

		// capture a frame from camera
		cap >> frame;


		// separate RGB channels
		std::vector<cv::Mat> channels;
		cv::split(frame,channels);
		cv::Mat R, G, B;
		R = channels[2];
		G = channels[1];
		B = channels[0];
		frame = R;


		// detect blobs in image
		pFile = fopen("testfile.txt","a");
		myThreshObj.set_image(frame);
		myThreshObj.detect_blobs();
		std::vector<cv::Point2f> imagePoints = myThreshObj.get_points();
		std::cout << "Number of blobs found -- " << imagePoints.size() << std::endl;
		std::cout << "Image Points: " << imagePoints << std::endl;
		fclose(pFile);


		// compute and display the fps MAVG
		std::cout << FPSclk.fps() << std::endl;

		//*
		// compute the KF state transition matrix (need dt first)
		double dt = double(FPSclk.elapsedTime)/1000;  //get the timestep for KF transition matrix
		KF.transitionMatrix = (cv::Mat_<float>(4,4) <<
				1, 0 , dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1);
		std::cout << KF.transitionMatrix << std::endl;

		// Kalman Filter predict & correct steps
		// pick off only the first point (for now)
		if (imagePoints.size() > 0) {

			// predict
			KF.predict();

			// correct
			z_k = (cv::Mat_<float>(kFilter.n_measurement,1) << imagePoints[0].x, imagePoints[0].y);
			KF.correct(z_k);

		} else {

			// predict
			KF.statePre = KF.transitionMatrix*KF.statePost;

			// correct (don't have measurement, so take a posteriori estimate to be equal to a priori)
			KF.statePost = KF.statePre;
		}

		// convert estimated state to point2f for plotting
		kFilter.estimate.x = KF.statePost.at<float>(0,0);
		kFilter.estimate.y = KF.statePost.at<float>(1,0);

		std::cout << "Before Prediction:\n" << KF.statePre << KF.statePost << std::endl;
		std::cout << "After Prediction:\n" << KF.statePre << KF.statePost << std::endl;
		//*/


		// print blobs on image
		channels[0] = R;
		channels[1] = R;
		channels[2] = R;
		cv::merge(channels,frame);
		myThreshObj.createBlobsImage(frame,cv::Scalar(0,0,255.0));
		cv::circle(frame, kFilter.estimate, 5, cv::Scalar(255.0,0,0), 3);
		cv::namedWindow("Blobs");
		cv::imshow("Blobs",frame);

	}

}

