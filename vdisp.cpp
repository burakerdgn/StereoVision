#include "vdisp.h"

#include <vector>

// Constructor
Vdisp::Vdisp()
{
}

// Destructor
Vdisp::~Vdisp()
{
}

// Processing
void Vdisp::ComputeVdisp(Mat& imgLeft , Mat& imgDisparity8U , int dispnum)
{
	vhist = Mat::zeros(imgDisparity8U.size().height, dispnum, CV_32F);
	edges = Mat::zeros(imgDisparity8U.size().height, dispnum, CV_32F);
	gray = Mat::zeros(imgDisparity8U.size().height, dispnum, CV_32F);

	//medianBlur ( imgDisparity8U , imgDisparity8U , 51);

	float value_ranges[] = { (float)0, (float)dispnum };
	const float* hist_ranges[] = { value_ranges };
	int channels[] = { 0 };
	int histSize[] = { dispnum };
	
	
	// Compute V-disparity with histogram method
	
	for (int i = 0; i < imgDisparity8U.rows; i++)
		    {
			    tmpImageMat = imgDisparity8U.row(i);
		    	vhist.row(i).copyTo(tmpHistMat);

		    	cv::calcHist(&tmpImageMat, 1, channels, cv::Mat(), tmpHistMat, 1, histSize, hist_ranges, true, false); // normal channel is 1

		    	vhist.row(i) = tmpHistMat.t() / (float)imgDisparity8U.rows;
		    }
		    
	vector<cv::Point> point;

    point.push_back(cv::Point(0, 0));  //point 1

	point.push_back(cv::Point(dispnum/4, 0));  //point 2


	point.push_back(cv::Point(dispnum/4, vhist.rows));  //point 3
	point.push_back(cv::Point(0,vhist.rows ));  //point 4

	cv::fillConvexPoly(vhist,               //Image to be drawn on
	 point,                 //C-Style array of points
	 cv::Scalar(255, 0, 0),  //Color , BGR form
	 cv::LINE_AA,             // connectedness, 4 or 8
	 0);            // Bits of radius to treat as fraction
		 
	 // Convert disparity to a form easy for visualization
     vhist.convertTo(vhist, CV_8U, 255);
     cv::applyColorMap(vhist, vhist, cv::COLORMAP_JET);


     //convert image to gray scale
     cv::cvtColor(vhist, gray, cv::COLOR_BGR2GRAY);

	  //Edge Detector
	  //cv::Canny(gray, edges, lowThreshold, lowThreshold *3);
      cv::threshold(gray, edges, 40, 255, cv::THRESH_BINARY);

	  cv::HoughLinesP(edges, lines, 1, CV_PI / 180,5, 50, 100);

	  Horizonline(imgLeft, imgDisparity8U);
	  //imshow("obj1", imgLeft);

}

void Vdisp::Horizonline(Mat& imgLeft, Mat& imgDisparity8U)
{
	//Draw lines
	if (!lines.empty())
		    {
				//cout << "there is a line" << endl ;
				for(int i = 0; i < lines.size(); i++)
				{
				    l = lines[i];
					//if the slope = infinity we can't calculate linear equation,so we must ignore the line
					if((abs(l[2] - l[0])) > 0){
						line(vhist, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
						break;
					}
				}

		    }
		    
	ComputeImages(imgLeft, imgDisparity8U);
	//imshow("obj",imgLeft);
}

void Vdisp::ComputeImages(Mat& imgLeft, cv::Mat& imgDisparity8U)
{

	medianBlur ( imgDisparity8U , imgDisparity8U , 51);
	//obj_img = imgLeft;
	imgLeft.copyTo(main_img);
	
	main_img.copyTo(obj_img);
	
	main_img.copyTo(grnd_img);
	
	
	
	//imgLeft.copyTo(main_img);
	//imgDisparity8U.copyTo(img_disparity);
	
	slope = (float)((l[3]) - (l[1])) / (float)(l[2] - l[0]); // calculate the angle of the line
	var = (float)((l[3]) - (slope * l[2])); 
	
	//cout << "slope:" << slope << " var:" << var << endl;
 
	for(int r=(grnd_img.rows-1) ; r >= 0 ; r--){  // how to find horizon line
		   
	disp_val = (r-var)/slope ;
	
	for(int c=0 ; c<grnd_img.cols ; c++){
		
		if (r >var){
			
		if ((disp_val) < (int)imgDisparity8U.at<uchar>(r,c)){
				
			grnd_img.at<uchar>(r,c) = 0;
				
				}
		else{
			obj_img.at<uchar>(r,c) = 0;
			}	
		}	
		else{
			grnd_img.at<uchar>(r,c) = 0;
		}	  
			  }
			}	
	//imshow("mediablur", main_img);
	imshow("main", imgLeft);
	imshow("obj", obj_img);
	imshow("grd", grnd_img);
	imshow("vDisp", vhist);
	//img_disparity = imgDisparity8U;
	//Display();
}


void Vdisp::Display()
{
	//imshow("vDisp", vhist);
	//imshow("Edges", edges);
	//imshow( "Disparity", img_disparity );
	//imshow("grd", grnd_img);
	//imshow("obj", obj_img);
}















