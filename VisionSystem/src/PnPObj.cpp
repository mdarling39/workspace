/*
 * PnPObj.cpp
 *
 *  Created on: Feb 18, 2013
 *      Author: michaeldarling
 */

#include "PnPObj.h"


// Default constructor
PnPObj::PnPObj() {

	// Initialize transformation matricies
	CV2B.create(3,3,CV_64F);
	CV2B.setTo(cv::Scalar(0));
	CV2B.at<double>(0,2) = 1.0;
	CV2B.at<double>(1,0) = 1.0;
	CV2B.at<double>(2,1) = 1.0;
	cv::transpose(CV2B,B2CV); 		// CV2B and B2CV convert between OpenCV

	// Initialize rvec_guess and tvec_guess
	// (assume directly behind by 100 inches)
	rvec_guess.create(3,1,CV_64F);
	tvec_guess.create(3,1,CV_64F);
	double initial_guess[6] = {0,0,100,0,0,0};
	PnPObj::setGuess(initial_guess);

	// Set rvec and tvec equal to guesses
	PnPObj::resetGuess();

	// Get rotation matrix for initial guess
	cv::Rodrigues(rvec,rotMat);

	// Compute Euler angles (and generate B_rotMat)
	B_tvec.create(3,1,CV_64F);
	PnPObj::computeEuler();

	// Initialize axesPoints vector
	axesPoints.push_back(cv::Point3f(0,0,0));
	axesPoints.push_back(cv::Point3f(AXES_LN,0,0));
	axesPoints.push_back(cv::Point3f(0,AXES_LN,0));
	axesPoints.push_back(cv::Point3f(0,0,AXES_LN));
}



// set guess
void PnPObj::setGuess(double arr[6]) {
	tvec_guess.at<double>(0,0) = arr[0]*IN2MM;
	tvec_guess.at<double>(1,0) = arr[1]*IN2MM;
	tvec_guess.at<double>(2,0) = arr[2]*IN2MM;
	rvec_guess.at<double>(0,0) = arr[3]*DEG2RAD;
	rvec_guess.at<double>(1,0) = arr[4]*DEG2RAD;
	rvec_guess.at<double>(2,0) = arr[5]*DEG2RAD;
}



// reset the initial guess
void PnPObj::resetGuess() {
	rvec = rvec_guess;
	tvec = tvec_guess;
}



// compute Euler angles and compute B_rotMat
void PnPObj::computeEuler() {
	cv::Rodrigues(rvec,rotMat);			// update rotation matrix
	B_rotMat = CV2B * rotMat * B2CV;	// change the rotation matrix from CV convention to RHS body frame
	theta = asin(-B_rotMat.at<double>(2,0));				// get the Euler angles
	psi = asin(B_rotMat.at<double>(1,0) / cos(theta));
	phi = asin(B_rotMat.at<double>(2,1) / cos(theta));
	B_tvec.at<double>(0,0) = tvec.at<double>(2,0);			// get the body translation vector
	B_tvec.at<double>(1,0) = tvec.at<double>(0,0);
	B_tvec.at<double>(2,0) = tvec.at<double>(1,0);
}



// set new Euler angles and recompute rotMat and rvec
void PnPObj::set_Euler(double phiIN, double thetaIN, double psiIN) {

	// set Euler angles
	phi = phiIN;
	theta = thetaIN;
	psi = psiIN;

	// reconstruct rotation matrix: from object frame to camera frame
	B_rotMat.at<double>(0,0) = cos(theta)*cos(psi);
	B_rotMat.at<double>(0,1) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
	B_rotMat.at<double>(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
	B_rotMat.at<double>(1,0) = cos(theta)*sin(psi);
	B_rotMat.at<double>(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
	B_rotMat.at<double>(1,2) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
	B_rotMat.at<double>(2,0) = -sin(theta);
	B_rotMat.at<double>(2,1) = sin(phi)*cos(theta);
	B_rotMat.at<double>(2,2) = cos(phi)*cos(theta);

	// recompute rvec from new rotation matrix
	rotMat = B2CV * B_rotMat * CV2B;	// convert body rotation matrix
	cv::Rodrigues(rotMat,rvec);			// to OpenCV rotation matrix
}



// load camera properties from XML/YML file
bool PnPObj::setCamProps(const std::string filename) {

	// Open xml file with camera properties
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;

#if DEBUG_MODE
	// write to standard output
	std::cout << "\nCamera properties read from:\n" << filename << std::endl;
	std::cout << "\ncameraMatrix:\n" << cameraMatrix << std::endl;
	std::cout << "\ndistCoeffs:\n" << distCoeffs << std::endl;
#endif /* DEBUG_MODE */

	fs.release();
	return 1;
}



// set image points
void PnPObj::setImagePoints(std::vector<cv::Point2f> imagePointsIN) {
	imagePoints = imagePointsIN;
}



// get image points
std::vector<cv::Point2f> PnPObj::getImagePoints() {
	return imagePoints;
}



// set object points
bool PnPObj::setModelPoints(const char* pointsFilename) {
	std::ifstream pointsFile;
	pointsFile.open(pointsFilename);
	if (!pointsFile.is_open()) {
		std::cout << "Could not open points file" << std::endl;
		pointsFile.close();
		exit(-1);
	} else {
		float p1, p2, p3;
		while (pointsFile >> p1 >> p2 >> p3) {
			modelPoints.push_back(cv::Point3f(p1,p2,p3));
		}
	}
	pointsFile.close();

#if DEBUG_MODE
	std::cout << "\nModel points read from:\n" << pointsFilename << std::endl;
	std::cout << "\nmodelPoints:\n" << modelPoints << std::endl;
#endif /* DEBUG_MODE */

	return 1;
}



// solve PnP problem
void PnPObj::solve() {
	cv::solvePnP(modelPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, 1, CV_EPNP);

	// compute Euler angles and compute B_rotMat
	PnPObj::computeEuler();

	// reproject points and compute reprojection error
	PnPObj::projectAll();
	PnPObj::scaledReprojError();
}



// project all points
void PnPObj::projectAll() {

	cv::projectPoints(modelPoints, rvec, tvec, cameraMatrix, distCoeffs, projImagePoints);
	cv::projectPoints(axesPoints, rvec, tvec, cameraMatrix, distCoeffs, projAxesPoints);

}



// compute scaled reprojection error
void PnPObj::scaledReprojError() {
	// Computes the scaled reprojection error.  Equal to the sum of the distances between the image
	// points and the reprojected image points divided by the maximum distance between image points

	// Sum up the shortest-distance between image points and reprojected image points
	double diff[NO_LEDS], sum;
	for (int i=0; i<NO_LEDS; i++) {
		diff[i] = cv::norm(imagePoints[i] - projImagePoints[i]);
	}
	sum = std::accumulate(diff, diff+NO_LEDS, 0.0);

	// compute the distance between all combinations of points, and find the maximum
	cv::Mat distances(NO_LEDS,NO_LEDS,CV_64F,cv::Scalar(0));
	for (int i = 0; i < NO_LEDS; i++) {
		for (int j = 0; j < NO_LEDS; j++){
			if (i <= j) continue;
			distances.at<double>(i,j) = cv::norm(imagePoints[i] - imagePoints[j]);
		}
	}

	double maxVal = *std::max_element(distances.begin<double>(),distances.end<double>());
	scaledReprojErr = sum/(maxVal);

}



// get scaledReprojError
double PnPObj::getScaledReprojError() {
	return scaledReprojErr;
}



// get state
std::vector<double> PnPObj::getState() {
	std::vector<double> state(6);
	state[0] = tvec.at<double>(2,0);
	state[1] = tvec.at<double>(0,0);
	state[2] = tvec.at<double>(1,0);;
	state[3] = phi;
	state[4] = theta;
	state[5] = psi;
	return state;
}



// draw over frame
void PnPObj::drawOverFrame(cv::Mat src, cv::Mat &out) {
	char pointLabel[1];
	cv::Point2f pointLabelLoc;
	for (int i=0; i<NO_LEDS; i++) {
		cv::circle(src,imagePoints[i], 5, cv::Scalar(0,0,255), -1);
		sprintf(pointLabel,"%d",i+1);
		pointLabelLoc = imagePoints[i] - cv::Point2f(8,8);
		putText(out, pointLabel, pointLabelLoc, cv::FONT_HERSHEY_PLAIN,
				1.5, cv::Scalar(255,255,255), 1);

		cv::circle(src,projImagePoints[i], 3, cv::Scalar(0,255,0), 3);
		cv::line(src,projAxesPoints[0], projAxesPoints[1], cv::Scalar(0,0,255), 2);
		cv::line(src,projAxesPoints[0], projAxesPoints[2], cv::Scalar(0,255,0), 2);
		cv::line(src,projAxesPoints[0], projAxesPoints[3], cv::Scalar(255,0,0), 2);
	}
	cv::Point2f textPt1(30, src.rows-40);
	cv::Point2f textPt2(30, src.rows-20);
	char stateText1[100], stateText2[100];
	sprintf(stateText1," dx: %8.2f    dy: %8.2f   dz: %8.2f",
			B_tvec.at<double>(0,0)*MM2IN,
			B_tvec.at<double>(1,0)*MM2IN,
			B_tvec.at<double>(2,0)*MM2IN);
	sprintf(stateText2,"phi: %8.2f  theta: %8.2f  psi: %8.2f",phi*RAD2DEG,theta*RAD2DEG,psi*RAD2DEG);
	putText(out, stateText1, textPt1, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1.2);
	putText(out, stateText2, textPt2, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1.2);
}



// correlate imagePoints to modelPoints
void PnPObj::correlatePoints(int callNo) {
	// Assumes 5 LEDs numbered 1-5:
	// Order Found	LED ID #		Description
	// point1 	= 	[0] or [4] 		First wingtip found
	// point2 	= 	[1] or [3] 		First horizontal found (same side as point1)
	// point3 	= 	[4] or [0] 		Opposite wing of point1
	// point4 	= 	[3] or [1] 		Horizontal opposite side of point1
	// point5 	= 	[2] 			Vertical tail

	// create temporary variables to hold points while things get moved around
	std::vector<cv::Point2f> bufferImagePoints = imagePoints, newImagePoints = imagePoints;

	if(callNo > 3) std::cout << "ledFinder only supports a maximum of 3 calls!" << std::endl;

	// Compute the mean pixel location of the LEDs
	cv::Point2f zero(0.0f, 0.0f);
	cv::Point2f sum  = std::accumulate(bufferImagePoints.begin(), bufferImagePoints.end(), zero);
	cv::Point2f meanPoint(sum.x / bufferImagePoints.size(), sum.y / bufferImagePoints.size());

	// Compute distances from mean point and find index of farthest point
	int point1 = 0;
	double distFromMean, maxDistFromMean = 0;
	for (int i=0; i<NO_LEDS; i++) {
		distFromMean = cv::norm(cv::Mat(bufferImagePoints[i]) - cv::Mat(meanPoint));

		point1 = (distFromMean > maxDistFromMean) ? i : point1;
		maxDistFromMean = (distFromMean > maxDistFromMean) ? distFromMean : maxDistFromMean;
	}

	// Determine if this is a right or left wingtip
	char firstWing;
	if (bufferImagePoints[point1].x < meanPoint.x) {
		// left wingtip
		newImagePoints[0] = bufferImagePoints[point1];
		firstWing = 'L';
	} else {
		// right wingtip
		newImagePoints[4] = bufferImagePoints[point1];
		firstWing = 'R';
	}

	// Find the closest and farthest points from first wingtip and make them the
	// horizontal of the same side and opposite wingtip, respectively
	int point2 = 0, point3;
	double distFrom1, minDistFrom1 = INFINITY;
	std::vector<std::pair<double,int> > point3Pairs;

	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1) continue; // Can't reuse points

		// find point2 (horizontal on same side as point1 wingtip)
		distFrom1 = cv::norm(cv::Mat(bufferImagePoints[i]) - cv::Mat(bufferImagePoints[point1]));
		point2 = (distFrom1 < minDistFrom1) ? i : point2;
		minDistFrom1 = (distFrom1 < minDistFrom1) ? distFrom1 : minDistFrom1;


		// find point3 (wingtip opposite of point1)
		// create a vector of distances from point 1 that can be sorted later
		point3Pairs.push_back(std::make_pair(i,distFrom1));
	}
	// sort by descending distance from point 1
	std::sort(point3Pairs.begin(), point3Pairs.end(), PnPObj::pairComparator);

	if (callNo==1) {
		// farthest point
		point3 = point3Pairs[0].first;
	} else if (callNo==2) {
		// second farthest point
		point3 = point3Pairs[1].first;
	} else if (callNo==3) {
		// third farthest point
		point3 = point3Pairs[2].first;
	}
	switch (firstWing) {
	case 'L':
		newImagePoints[1] = bufferImagePoints[point2];
		newImagePoints[4] = bufferImagePoints[point3];
		break;
	case 'R':
		newImagePoints[3] = bufferImagePoints[point2];
		newImagePoints[0] = bufferImagePoints[point3];
		break;
	default:
		std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
		break;
	}

	// Compute the angle of the wings and find which point (point4), when matched with the
	// known horizontal, most closely matches the wing angle.
	double wingSlope, wingAngle;
	wingSlope = (newImagePoints[4].y - newImagePoints[0].y)/(newImagePoints[4].x - newImagePoints[0].x);
	wingAngle = atan(wingSlope);

	int point4 = 0;
	double slope, angle, absAngleDiff, minAbsAngleDiff = INFINITY;
	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1 || i==point2 || i== point3) continue;  // Can't reuse previous points
		switch (firstWing) {
		case 'L':
			slope = (newImagePoints[1].y - bufferImagePoints[i].y) / (newImagePoints[1].x - bufferImagePoints[i].x);
			break;
		case 'R':
			slope = (bufferImagePoints[i].y - newImagePoints[3].y) / (bufferImagePoints[i].x - newImagePoints[3].x);
			break;
		default:
			std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
			break;
		}

		angle = atan(slope);
		absAngleDiff = std::abs(angle - wingAngle);

		point4 = (absAngleDiff < minAbsAngleDiff) ? i : point4;
		minAbsAngleDiff = (absAngleDiff < minAbsAngleDiff) ? absAngleDiff : minAbsAngleDiff;
	}

	switch (firstWing) {
	case 'L':
		newImagePoints[3] = bufferImagePoints[point4];
		break;
	case 'R':
		newImagePoints[1] = bufferImagePoints[point4];
		break;
	default:
		std::cout << "Error evaluating if the first LED is left or right wingtip" << std::endl;
		break;
	}


	// The final LED (point5) must be the tail
	int point5;
	for (int i=0; i<NO_LEDS; i++) {
		if (i==point1 || i ==point2 || i==point3 || i==point4) {
			continue;
		} else {
			point5 = i;
			break;
		}
	}
	newImagePoints[2] = bufferImagePoints[point5];

	imagePoints = newImagePoints;
}
