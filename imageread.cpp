
// OpenCV includes
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "vdisp.h"

#include <stdint.h>
#include <unistd.h>

using namespace std;


#define MAX_DISP 100 /*250*/

cv::Mat computeVDisparity(cv::Mat img);
cv::Mat image_left;
cv::Mat image_right;

int main(int argc, char **argv) { 
	
	char key = ' ';
   while (key != 'q') {
   
	Vdisp vp ;
	
	image_left = cv::imread("leftzed.png", CV_LOAD_IMAGE_COLOR);
	image_right = cv::imread("rightzed.png", CV_LOAD_IMAGE_COLOR);
	
	cv::Mat ground = cv::Mat::zeros(image_right.size().height, image_left.size().width, CV_8UC1);

	// create the image in which we will save our disparities
	cv::Mat imgDisparity8U = cv::Mat(image_left.rows, image_left.cols, CV_8UC1);
	
	imgDisparity8U = cv::imread("dispzed.png", CV_LOAD_IMAGE_COLOR);
	        
	cvtColor(imgDisparity8U, imgDisparity8U, cv::COLOR_RGB2GRAY);
	
	image_left.copyTo(ground); 

	cvtColor(ground, ground, cv::COLOR_RGB2GRAY);

    
    // Loop until 'q' is pressed
   // char key = ' ';
   // while (key != 'q') {
	        
			//comment out this line to use this function and comment "for loop", which calculates V-Histogram
			//vhist = computeVDisparity(imgDisparity8U);  //in this method you have too much noise, but  road profile is more recognizable
			
			//imshow("obj", ground);
			
			//imshow("obj1", ground);

            vp.ComputeVdisp(ground,imgDisparity8U,MAX_DISP);
            
			
            // Handle key event
            key = cv::waitKey(10);
        
    }
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat computeVDisparity(cv::Mat img)
{
	int maxDisp = 256;
	cv::Mat vDisp(img.rows, maxDisp, CV_8U, cv::Scalar(0));
	for (int u = 0; u < img.rows; u++) {
		uchar* ptrA = img.ptr<unsigned char>(u);
		for (int v = 0; v < img.cols; v++) {
			if (ptrA[v] != 0 && ptrA[v] != 255)
			{
				vDisp.at<unsigned char>(u, ptrA[v]) += 1;
			}
		}
	}
	return vDisp;
}
