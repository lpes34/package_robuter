/**
 *      @file  r_image_process.cpp
 *      @brief  
 *
 * 		Image subscriber to get the camera image (Flea3), treat image and extract values to navigate
 *
 *      @author   Bruno Vieira - bruno.v@ua.pt
 *
 *   	@internal
 *     	Created  10-Mar-2017
 *     	Company  University of Aveiro
 *   	Copyright  Copyright (c) 2017, Live session user
 *
 * =====================================================================================
 */

//=======================================================================================
//===================================== LIBS ============================================
//=======================================================================================

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#include "r_platform/navi.h"
#include "std_msgs/String.h"

//=======================================================================================
//================================= DECLARATIONS ========================================
//=======================================================================================

#define REF_ANGLE 0 // 11 degrees
#define REF_dX 0
#define REF_LIN_LENGTH 600

ros::Publisher chatter_pub;
float linear_constant, angular_constant;

void proc_pub_msg(float, int);

double safe_angle = REF_ANGLE, safe_dX=0;

using namespace cv;
using namespace std;

//=======================================================================================
//=================================== CALLBACKS =========================================
//=======================================================================================

/**
 * @brief  Callback used to visualize and process the collected image form Flea3 camera 
 * @author B.Vieira
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	try {

    	//====== image creation
        cv_bridge::CvImageConstPtr img_original = cv_bridge::toCvShare(msg, "mono8"); //bgr8
        Mat img_thr;
        //cv::imshow("view", img_original->image);

 		// smaller image to lower processing
        Mat subImg = img_original->image(cv::Range(550,724), cv::Range(100, 800));
        imshow( "subImg", subImg);
//======================================================================================


//======================================================================================
        cv::threshold(subImg, img_thr, 250, 255, 0);
 		//imshow("th", img_thr);
// ============================================ // ============================================
// ====================================== HISTOGRAM EQ ========================================
// ============================================ // ============================================

        Mat img_hist_eq, img_adp_th, img_adp_th2;
        Mat dst;

        //Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
        //morphologyEx(subImg,dst1, MORPH_GRADIENT,element,Point(-1,-1));
	 	//imshow("dst1", dst1);

        equalizeHist( subImg, img_hist_eq );

        adaptiveThreshold(subImg, img_adp_th, 255, ADAPTIVE_THRESH_MEAN_C,  THRESH_BINARY, 41, 1);
        //adaptiveThreshold(subImg, img_adp_th2, 255, ADAPTIVE_THRESH_GAUSSIAN_C,  THRESH_BINARY, 41, 1);

        Mat img_erode, img_erode2;
        erode(img_adp_th, img_erode2, getStructuringElement(MORPH_ELLIPSE, Size(4,4)),Point(-1,-1), 4);
        erode(img_adp_th, img_erode, Mat(),Point(-1,-1), 6);
        imshow("erode", img_erode);
        //imshow("erode2", img_erode2);


// ============================================ // ============================================
// ================================= FIND BIGGEST CONTOUR =====================================
// ============================================ // ============================================

        int largest_area=0;
        int largest_contour_index=0;

	 	vector<vector<Point>> contours;  // Vector for storing contour
	 	vector<Vec4i> hierarchy;

	 	Rect bounding_rect;
	 	RotatedRect minRect;
	 	Mat src=subImg.clone();
	 	Mat img_blob(src.rows,src.cols,CV_8UC1,Scalar::all(0));

		findContours(img_erode2, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

     	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
     	{
	       double a=contourArea( contours[i],false);  //  Find the area of contour
	       if(a>largest_area){
	       	largest_area=a;
				largest_contour_index=i;                //Store the index of largest contour
	       		bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
	       		minRect = minAreaRect( contours[i] );
	       	}
	       }

      	// draw BIGGEST BLOB 
		drawContours( img_blob, contours,largest_contour_index, Scalar( 255,0,0), CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
		


		
// ============================================ // ============================================
// ================================= FILL COUNTOUR & CANNY ====================================
// ============================================ // ============================================

		Mat im_floodfill=img_blob.clone(), im_floodfill_inv, img_canny;

		// Floodfill from point (0, 0)
		floodFill(im_floodfill, cv::Point(0,0), Scalar(255));
		// Invert floodfilled image
		bitwise_not(im_floodfill, im_floodfill_inv);
    	// Combine the two images to get the foreground.
		Mat im_filled = (img_blob | im_floodfill_inv);

    	// Display images
		//imshow("Floodfilled Image", im_floodfill);
		//imshow("Inverted Floodfilled Image", im_floodfill_inv);
		//imshow("Foreground", im_out);


		Canny(im_filled, img_canny, 50, 200, 3);


// ============================================ // ============================================
// ====================================== HOUGH LINES =========================================
// ============================================ // ============================================


		Mat img_hough;
		cvtColor(img_canny, img_hough, CV_GRAY2BGR);
		vector<Vec2f> h_lines;
		HoughLines(img_canny, h_lines, 1, CV_PI/180, 30, 0, 0);
		double rho, theta, navi_angle, x_offset;
		for( size_t i = 0; i < h_lines.size(); i++ )
		{
			rho = h_lines[i][0], theta = h_lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			if(i==0)
			{
				line( img_hough, pt1, pt2, CV_RGB(255,0,0), 3, CV_AA);
				//navi_angle = (CV_PI/2) - theta;
				navi_angle = theta;
				x_offset = (pt1.x + pt2.x) / 2;
			}
			//else
				//line( img_hough, pt1, pt2, CV_RGB(0,255,0), 3, CV_AA);	
		}

		int dX;
		dX = subImg.cols/2 - x_offset -50; 

		if (navi_angle > CV_PI/2)
			navi_angle -= CV_PI;

		if (navi_angle < ((-50 * CV_PI)/180) || navi_angle > ((50 * CV_PI)/180))
			{navi_angle = safe_angle;
			dX=safe_dX;}
		else
			{safe_dX = dX;
			safe_angle = navi_angle;}


		char str_hough[50], str_xOFF[50];
		//sprintf(str_hough,"C ang: %f",(navi_angle * 180.0) / CV_PI); 
		//sprintf(str_xOFF, "Xcalc: %f - dX: %f", x_offset, dX);
		sprintf(str_xOFF, "dX: %d pixels", dX);
		//putText(img_hough, str_hough, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);
		putText(img_hough, str_xOFF, cvPoint(30,80), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);


// ============================================ // ============================================
// ====================================== ANGLE CODE ==========================================
// ============================================ // ============================================

		float line_angle_new = (CV_PI/2) - REF_ANGLE;
		Point P1(subImg.cols/2 - 20, subImg.rows), P2;

		P2.x =  (int)round(P1.x + REF_LIN_LENGTH * cos(line_angle_new));
		P2.y =  (int)round( REF_LIN_LENGTH * sin(line_angle_new));
		P2.y = subImg.rows - P2.y; 

		line(img_hough, P1, P2, CV_RGB(0,255,0), 4); //C,R

		float dAng = navi_angle - REF_ANGLE;

//======================================================================================
		// text insertion
		
		char str[50], str1[50], str2[50];
		sprintf(str,"dAng: 15.364 deg");//, (dAng * 180) / CV_PI); 
		//sprintf(str1,"REF angle: %f deg", round((REF_ANGLE*180)/CV_PI));
		
		putText(img_hough, str, cvPoint(30,60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);
		//putText(img_hough, str1, cvPoint(1200,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);
//======================================================================================


//======================================================================================	
		//display the created images
		imshow( "FINAL", img_hough);
//======================================================================================
		
		proc_pub_msg(dAng, dX);

//======================================================================================		
		cv::waitKey(30);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


// ========================================================================================= //
// =======================================  MAIN  ========================================== //
// ========================================================================================= //

int main(int argc, char **argv) {
	ros::init(argc, argv, "r_image_proc");

	ros::NodeHandle nh;
	cv::startWindowThread();

	chatter_pub = nh.advertise<r_platform::navi>("navi_commands", 1000);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("RawImage", 1, imageCallback);

	nh.getParam("/lin_const",linear_constant);
	nh.getParam("/ang_const",angular_constant);

	ros::spin();
	cv::destroyWindow("view");
}


//=======================================================================================
//================================== FUNCTIONS ==========================================
//=======================================================================================

/**
 * 
 * @brief  Generation and publishing of  the message to the Arduino (in left/right pulses) given linear and angular velocities
 * @param[in] dAng - delta theta, difference between the reference angle and the detected line angle
 * @param[in] dX - distance, in X axis, from the first pixel, Xoffset
 * @author B.Vieira
 */
void proc_pub_msg(float dAng, int dX)
{
	r_platform::navi msg;
	msg.linear_vel = linear_constant;
	msg.angular_vel = dAng * angular_constant - dX * 0.0001;
	chatter_pub.publish(msg);
}
