//============================================================================
// Name        : SimplePnP.cpp
// Author      : Michael Darling
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>


int main() {

	// Add multiple images to get 5 LEDs in the same frame
	cv::Mat img = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib1.jpg");
	cv::Mat img2 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib2.jpg");
	cv::Mat img3 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib3.jpg");
	cv::Mat img4 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib4.jpg");
	cv::Mat img5 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib5.jpg");
	cv::Mat img6 = cv::imread("/Users/michaeldarling/Dropbox/Thesis/OpenCV/Thresholding/Calib6.jpg");
	img = img + img2 + img3 + img4 + img5 + img6;

	// create key points
	cv::vector<cv::KeyPoint> keyPoints;
	cv::vector< cv::vector <cv::Point> > contours;
	cv::vector< cv::vector <cv::Point> > approxContours;


	// set blob parameters
	cv::SimpleBlobDetector::Params blobParams;
	blobParams.minThreshold = 200;
	blobParams.maxThreshold = 255;
	blobParams.thresholdStep = 5;
	blobParams.minDistBetweenBlobs = 10;
	blobParams.minArea = 5;
	blobParams.maxArea = 500;
	blobParams.blobColor = 255;  // I think this is red color
	blobParams.filterByArea = true;
	blobParams.filterByColor = true;
	blobParams.filterByInertia = false;
	blobParams.filterByConvexity = false;
	blobParams.filterByCircularity = false;


	// set up and create the detector using the parameters
	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(blobParams);
	blob_detector->create("SimpleBlob");

	// detect!
	blob_detector->detect(img, keyPoints);
	std::cout << (double)keyPoints.size() << " points detected" << std::endl;

	// display and draw the x y coordinates of the keypoints:
	cv::Mat blobs(img.size(),CV_8U,cv::Scalar(0));
	cv::drawKeypoints( blobs, keyPoints, blobs, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DEFAULT);

	// convert points to a Point2f vector
	std::vector<cv::Point2f> points2f;
	cv::KeyPoint::convert(keyPoints, points2f);
	std::cout << points2f << std::endl;


	cv::namedWindow("Image");
	cv::imshow("Image",img);
	cv::namedWindow("Blobs");
	cv::imshow("Blobs",blobs);
	cv::waitKey(0);

	return 0;

}
