//============================================================================
// Name        : LEDDetectorTest.cpp
// Author      : Michael Darling
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

// GLOBAL DEFINES
#define FOUNDBLOBS_TO_FILE
#define SAVEOFF_FRAMES
//#define DO_KALMAN_FILTER

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include "Threshold.hpp"
#include "KalmanFilter.hpp"
#include "FPSCounter.hpp"
#include "customblobdetector.hpp"

#ifdef FOUNDBLOBS_TO_FILE
#include <stdio.h>  // FORLINUX
FILE *pFile;
#endif // FOUNDBLOBS_TO_FILE

#ifdef SAVEOFF_FRAMES
#include <sys/types.h>
#include <sys/stat.h>
unsigned int frameCount = 0;
#endif // SAVEOFF_FRAMES


int main() {


	// initialize camera and capture some frames
	cv::Mat frame;
	cv::VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,640.0);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,480.0);
	// cap.set(CV_CAP_PROP_FPS,60);
	if (!cap.isOpened()) {
		return -1;
	}
	for(int i=0; i<20; i++) {
		cap >> frame;
	}


	// set blob parameters
	CustomBlobDetector::Params blobParams;

	// NO LONGER INCLUDED IN BLOBPARAMS
	//blobParams.minRepeatability = 1;
	//blobParams.maxThreshold = 256;
	//blobParams.thresholdStep = blobParams.maxThreshold - blobParams.minThreshold - 1;
	//blobParams.minDistBetweenBlobs = 8;

	blobParams.maxPoints = 10;
	blobParams.maxError = 0.30;
	blobParams.minArea = 1;
	blobParams.maxArea = 200;
	blobParams.minCircularity = 0.05;
	blobParams.maxCircularity = 1.1;
	blobParams.minInertiaRatio = 0.05;
	blobParams.maxInertiaRatio = 1.1;
	blobParams.minConvexity = 0.05;
	blobParams.maxConvexity = 1.1;
	blobParams.minThreshold = 250;


	blobParams.targetCircularity = 1.0;
	blobParams.targetInertiaRatio = 1.0;
	blobParams.targetConvexity = 1.0;
	blobParams.targetBlobColor = 255;

	blobParams.w_Circularity = 255;
	blobParams.w_InertiaRatio = 0;
	blobParams.w_Convexity = 0;
	blobParams.w_BlobColor = 255;

	blobParams.filterByError = false;
	blobParams.filterByArea = true;
	blobParams.filterByColor = true;
	blobParams.filterByCircularity = true;
	blobParams.filterByInertia = true;
	blobParams.filterByConvexity = true;




	// create a threshold object
	Threshold myThreshObj;
	myThreshObj.set_params(blobParams);


#ifdef DO_KALMAN_FILTER
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
	KF.processNoiseCov = cv::Mat::eye(kFilter.n_states,kFilter.n_states,kFilter.type)*(1e-3);

	// initialize covariance of measurement noise (R)
	KF.measurementNoiseCov = cv::Mat::eye(kFilter.n_measurement,kFilter.n_measurement,kFilter.type)*(1e-2);

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
#endif //DO_KALMAN_FILTER

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

	fprintf(pFile,"\n%15s%15s%15s%15s%15s%15s%15s%15s%15s\n","blobNo","TotalError","circularity","inertiaRatio",
			"convexity","blobColor","area","imageXpt","imageYpt");
	fclose(pFile);
#endif  //FOUNDBLOBS_TO_FILE

#ifdef SAVEOFF_FRAMES
	// clear the TestImages folder
	system("rm -r TestImages");
	if (mkdir("TestImages", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
		std::cout << "TestImages folder successfully cleared!" << std::endl;
	}
	// get an RGB frame for reference
	cv::imwrite("TestImages/frame_0.jpg", frame);
#endif // SAVEOFF_FRAMES






	// MAIN LOOP
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
		std::cout << "Average FPS:  " << FPSclk.fps() << std::endl;




#ifdef DO_KALMAN_FILTER
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
		kFilter.estimate.x = KF.statePre.at<float>(0,0);
		kFilter.estimate.y = KF.statePre.at<float>(1,0);

		std::cout << "Before Prediction:\n" << KF.statePre << KF.statePost << std::endl;
		std::cout << "After Prediction:\n" << KF.statePre << KF.statePost << std::endl;
		//*/


#endif //DO_KALMAN_FILTER


		// print blobs on image
		channels[0] = R;
		channels[1] = R;
		channels[2] = R;
		cv::merge(channels,frame);
		myThreshObj.createBlobsImage(frame,cv::Scalar(0,0,255.0));
		if (imagePoints.size() > 0) cv::circle(frame,imagePoints[0], 5 ,cv::Scalar(0,255,0), 3);
#ifdef DO_KALMAN_FILTER
		cv::circle(frame, kFilter.estimate, 5, cv::Scalar(255.0,0,0), 3);
#endif //DO_KALMAN_FILTER
		cv::namedWindow("Blobs");
		cv::imshow("Blobs",frame);
		cv::waitKey(1);   //need to wait to keep the image displayed

#ifdef SAVEOFF_FRAMES
		frameCount++;
		if (frameCount % 30 == 0) { // Save one in every 30 frames (~1 frame/sec)

			// add a Frame number label
			std::stringstream frameNoStr;
			frameNoStr << "Frame # " << frameCount;
			cv::putText(frame,frameNoStr.str(),cv::Point2f(20,20),FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255));

			// save off the file
			frameNoStr.str("");  // clear the string
			frameNoStr << "TestImages/frame_" << frameCount << ".jpg";
			cv::imwrite(frameNoStr.str(), frame);

		}
#endif // SAVEOFF_FRAMES

	}

}

