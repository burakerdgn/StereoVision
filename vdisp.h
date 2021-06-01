/**
 * Hochschule Esslingen 07.05.2020
 * Prof. Thao Dang , Burak Erdogan
 * Calculation of v-disparity
 */

#ifndef __VDISP__
#define __VDISP__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace std;
using namespace cv;

class Vdisp
{
	
private:

   /*! 
    *  Common variables for v-disparity calculation
    */
	vector<cv::Vec4i> lines; /*!< the lines that are produced by Hough trasnform */
	cv::Vec4i l;  /*!< v-disparity line */

	cv::Mat edges;  
	cv::Mat gray; /*!< gray image for v-disp */
	cv::Mat vhist; /*!< v-disparity image */

	cv::Mat main_img;
	cv::Mat obj_img;  /*!< object image */
	cv::Mat img_disparity; /*!< disparity image */
	cv::Mat grnd_img; /*!< ground image */ 
	
	/*! 
	 *  Variables of V-disparity line equation
	 */
	float slope; /*!< slope is the slope of the equation */
	float var;   /*!< slope is the slope of the equation */
	float disp_val ; /*!< slope is the slope of the equation */

	cv::Mat tmpImageMat, tmpHistMat;
		
public:

   // Constructor
   Vdisp();

   // Destructor
   ~Vdisp();
   
   /*! Compute V-disparity with Hough transform method
    * imgLeft is the main image
    * imgDisparity8U disparity image
    * dispnum max. disparity number
    */
   void ComputeVdisp(cv::Mat& imgLeft, cv::Mat& imgDisparity8U , int dispnum);
   
   /*!
    *  Find the v-disp line 
    */
   void Horizonline(cv::Mat& imgLeft, cv::Mat& imgDisparity8U);
   
   
   /*!
    * Compute ground and object images
    */
   void ComputeImages(cv::Mat& imgLeft, cv::Mat& imgDisparity8U);
   
private:

	/*!
	 *  Display images
	 */
	void Display();
	
};

#endif
