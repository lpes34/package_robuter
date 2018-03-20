/**
 *      @file  r_exp_results.cpp
 *      @brief  
 *
 * 		This node was developed to perform experimental results on the platform. 
 *		It's an image subscriber to get the camera image (Flea3), treat it and extract angles to measure the system's deviation.
 *		Delta Theta will be published so it can be visualized in RQT .graph
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

#include "std_msgs/String.h"
#include "r_platform/navi.h"

//=======================================================================================
//================================= DECLARATIONS ========================================
//=======================================================================================

#define REF_ANGLE 0.0 // 11 degrees
#define REF_dX 100
#define REF_LIN_LENGTH 600

ros::Publisher chatter_pub;

using namespace cv;
using namespace std;

//=======================================================================================
//=================================== CALLBACKS =========================================
//=======================================================================================

/**
 * @brief  Callback used to visualize and process the collected image from Flea3 camera 
 * @author B.Vieira
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	try {

    	//====== image creation
        cv_bridge::CvImageConstPtr img_original = cv_bridge::toCvShare(msg, "mono8"); //bgr8
        Mat img_thr;
        //cv::imshow("view", img_original->image);

 		// smaller image to lower processing
        Mat subImg = img_original->image(cv::Range(300,724), cv::Range(164, 800));
        imshow( "subImg", subImg);
//======================================================================================


//======================================================================================
        cv::threshold(subImg, img_thr, 150, 255, 0);
 		imshow("th", img_thr);
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

	//imshow("adp_TH", img_adp_th);

        Mat img_erode, img_erode2, img_dilate, img_close;
        erode(img_adp_th, img_erode2, getStructuringElement(MORPH_ELLIPSE, Size(4,4)),Point(-1,-1), 4);

        dilate(img_adp_th, img_dilate, Mat(), Point(-1,-1), 5);

        erode(img_adp_th, img_close, Mat(),Point(-1,-1), 5);
       // imshow("close", img_close);
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
		
		//imshow("img_blob", img_blob);

// ============================================ // ============================================
// ================================ FIND FIRST WHITE POINTS ===================================
// ============================================ // ============================================

		int count1=0, count2=0;
		for (int col=0;col<img_blob.cols;col++) 
		{
			if(img_blob.at<uchar>(200, col) != 0)
			{
				count1 = col;
				break;
			} 
		} 

		for (int col=0;col<img_blob.cols;col++) 
		{
			if(img_blob.at<uchar>(400, col) != 0)
			{
				count2 = col;
				break;
			} 
		} 


		int count3=0, count4=0;
		for (int col=img_blob.cols; col>0;col--) 
		{
			if(img_blob.at<uchar>(200, col) != 0)
			{
				count3 = col;
				break;
			} 
		} 

		for (int col=img_blob.cols; col>0;col--) 
		{
			if(img_blob.at<uchar>(400, col) != 0)
			{
				count4 = col;
				break;
			} 
		} 

		

		cv::Mat img_rgb(subImg.size(), CV_8UC3);
		// convert grayscale to color image
		cv::cvtColor(img_blob, img_rgb, CV_GRAY2RGB);



		double angle1 = atan2(400 - 200, count2 - count1), angle2 = atan2(400 - 200, count4 - count3) ;

// ============================================ // ============================================
// ====================================== REF ANGLE CODE ==========================================
// ============================================ // ============================================

		float line_angle_new = (CV_PI/2) - REF_ANGLE;
		Point P1(subImg.cols/2 - 20, subImg.rows), P2, P_l1, P_l2, P_r1, P_r2;

		P2.x =  (int)round(P1.x + REF_LIN_LENGTH * cos(line_angle_new));
		P2.y =  (int)round( REF_LIN_LENGTH * sin(line_angle_new));
		P2.y = subImg.rows - P2.y; 

		//line(img_hough, P1, P2, CV_RGB(0,0,255), 4); //C,R
		line(img_rgb, P1, P2, CV_RGB(0,0,255), 4);

		P_l1.y= 200;
		P_l1.x= count1;
		P_l2.y= 400;
		P_l2.x= count2;

		P_r1.y= 200;
		P_r1.x= count3;
		P_r2.y= 400;
		P_r2.x= count4;

		line(img_rgb, P_l1, P_l2, CV_RGB(0,200,200), 4);
		line(img_rgb, P_r1, P_r2, CV_RGB(0,200,200), 4);

		float dAng = (angle1 + angle2) / 2 - CV_PI/2;

//======================================================================================
		// text insertion
		
		char str[50], str1[50], str2[50];

		sprintf(str,"dAng: %f deg", (dAng * 180) / CV_PI); 
		putText(img_rgb, str, cvPoint(30,90), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255,0,0), 1, CV_AA);
		
		sprintf(str1,"C ang: %f | %f ", (angle1 * 180.0) / CV_PI, (angle2 * 180.0) / CV_PI); 
		putText(img_rgb, str1, cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);

  //sprintf(str_hough2,"Average: %f",(final_angle * 180.0) / CV_PI); 
		sprintf(str2,"AVG: %f ",( (angle1 + angle2) / 2) * 180.0 / CV_PI); 
		putText(img_rgb, str2, cvPoint(30,60), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,255,0), 1, CV_AA);
		//sprintf(str1,"REF angle: %f deg", round((REF_ANGLE*180)/CV_PI));
		
		
//======================================================================================


//======================================================================================	
		//display the created images
		imshow("RBG", img_rgb);
//======================================================================================
		
		//message publishing
		r_platform::navi msg;
		msg.dTheta = (dAng * 180) / CV_PI;
		chatter_pub.publish(msg);

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

	chatter_pub = nh.advertise<r_platform::navi>("exp_results", 1000);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("RawImage", 1, imageCallback);

	ros::spin();
	cv::destroyWindow("view");
}



