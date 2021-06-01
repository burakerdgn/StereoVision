 
#include "vdisp.h" 

 // ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


//#include "vdisp.h"

#include <stdint.h>
#include <unistd.h>


using namespace sl;
using namespace std;


#define MAX_DISP 100 /*250*/

/** Asisstant functions
 */
cv::Mat slMat2cvMat(sl::Mat& input); /** Transform zed images to OpenCV image */
cv::Mat computeVDisparity(cv::Mat img); /**Compute V-disparity */


/** 
 * Function for displaying image type and channel
 */
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int main(int argc, char **argv) {


	//
	
	Vdisp vp;
	
    /**
     *  Create a ZED camera object
     */
    Camera zed;

    /**
     *  Set configuration parameters
     */
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;/*RESOLUTION_HD1080;*/
    init_params.depth_mode = DEPTH_MODE_ULTRA;
    init_params.coordinate_units = UNIT_METER;

    if (argc > 1) init_params.svo_input_filename.set(argv[1]);

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    
    /**
     *  Set runtime parameters after opening the camera
     */
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;
    
    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself

     // Prepare new image size to retrieve new resolution images
     
     
    Resolution image_size = zed.getResolution();
     int new_width = image_size.width / 2 ; // / 2 olucak
     int new_height = image_size.height / 2;
    
    sl::Mat imgLeft(new_width, new_height, MAT_TYPE_8U_C1);
    cv::Mat imgLeft_ocv = slMat2cvMat(imgLeft);

    sl::Mat imgRight(new_width, new_height, MAT_TYPE_8U_C1);
    cv::Mat imgRight_ocv = slMat2cvMat(imgRight);
    
    sl::Mat depth_map(new_width, new_height, MAT_TYPE_32F_C1);
    cv::Mat imgdepth = slMat2cvMat(depth_map);
    
    
    
    /**
     * Parameters for calculating disparity image with equation  -> depth = (base line * focal length)/ disparity
     */
	CameraInformation myCam = zed.getCameraInformation(Resolution(new_width, new_height));	
	double focal_length_px = myCam.calibration_parameters.left_cam.fx;
	double baseline_m = sqrt(myCam.calibration_parameters.T[0]*myCam.calibration_parameters.T[0] + 
		myCam.calibration_parameters.T[1]*myCam.calibration_parameters.T[1] + 
		myCam.calibration_parameters.T[2]*myCam.calibration_parameters.T[2]);
		 
	//cout << focal_length_px << endl;
	//cout << baseline_m << endl;

    cv::Mat vhist = cv::Mat::zeros(new_height, MAX_DISP, CV_32F);
    cv::Mat tmpImageMat, tmpHistMat;

	float value_ranges[] = { (float)0, (float)MAX_DISP };
	const float* hist_ranges[] = { value_ranges };
	int channels[] = { 0 };
	int histSize[] = { MAX_DISP };


	// create the image in which we will save our disparities
	cv::Mat imgDisparity16S = cv::Mat(imgLeft_ocv.rows, imgLeft_ocv.cols, CV_16S);
	cv::Mat imgDisparity8U = cv::Mat(imgLeft_ocv.rows, imgLeft_ocv.cols, CV_8UC1);
	cv::Mat imgDisparity8U1 = cv::Mat(imgLeft_ocv.rows, imgLeft_ocv.cols, CV_8UC1);
	
	//cout << "rows: " << imgLeft_ocv.rows << " cols: " << imgLeft_ocv.cols << endl;

    cv::Mat gray = cv::Mat::zeros(new_height, MAX_DISP, CV_32F);
    cv::Mat edges = cv::Mat::zeros(new_height, MAX_DISP, CV_32F);
    cv::Mat object = cv::Mat::zeros(new_height, new_height, CV_8UC1);
	cv::Mat ground = cv::Mat::zeros(new_height, new_height, CV_8UC1);

    /**
     *  Call the constructor for StereoSGBM
     */
	int ndisparities = (MAX_DISP/16)*16+16;   /* Range of disparity */
	int SADWindowSize = 3; /* Size of the block window. Must be odd */
    cv::Ptr<cv::StereoSGBM> sbm = cv::StereoSGBM::create(ndisparities, SADWindowSize);
	int cn = imgLeft_ocv.channels();
    sbm->setPreFilterCap(63);
	sbm->setP1(8 * cn * SADWindowSize*SADWindowSize);
	sbm->setP1(32 * cn * SADWindowSize*SADWindowSize);
	sbm->setUniquenessRatio(10);
	sbm->setSpeckleWindowSize(100);
	sbm->setSpeckleRange(32);
	sbm->setDisp12MaxDiff(1);
	sbm->setNumDisparities(ndisparities);
	sbm->setMinDisparity(0);


    // extreme values
	double minVal; double maxVal;

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == SUCCESS) {

		   /**
		    * read zed images
		    */
	       zed.retrieveMeasure(depth_map, MEASURE_DEPTH, MEM_CPU, new_width, new_height); // Retrieve DISPARITY
           zed.retrieveImage(imgLeft, VIEW_LEFT_GRAY , MEM_CPU, new_width, new_height);
           zed.retrieveImage(imgRight, VIEW_RIGHT_GRAY , MEM_CPU, new_width, new_height);
           
           imgLeft_ocv.copyTo(ground); 

		   //cvtColor(ground1, ground1, cv::COLOR_RGB2GRAY);

            //Calculate the disparity image
            sbm->compute(imgLeft_ocv, imgRight_ocv, imgDisparity16S);
            

            minMaxLoc(imgDisparity16S, &minVal, &maxVal);
            
            //Display it as a CV_8UC1 image
	        imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 1.0/16.0); //255 / (maxVal - minVal));
	        
	        //cvtColor(ground1, ground1, cv::COLOR_RGB2GRAY);
	        
	        //cvtColor(imgDisparity8U, imgDisparity8U, cv::COLOR_RGB2GRAY);
	        
			//comment out this line to use this function and comment "for loop", which calculates V-Histogram
			//vhist = computeVDisparity(imgDisparity8U);  //in this method you have too much noise, but  road profile is more recognizable

			double min1;
			double max1;
			double mean1;
			
			threshold(imgdepth, imgdepth,  10000, 0, cv::THRESH_TOZERO_INV);
			threshold(imgdepth, imgdepth, -10000, 0, cv::THRESH_TOZERO);
			divide(baseline_m*focal_length_px, imgdepth, imgdepth);
			
			minMaxLoc(imgdepth, &min1, &max1);
			
			//cv::Scalar tempVal = mean( imgdepth );
            //mean1 = tempVal.val[0];
			//cout << "mean:" << mean1 << endl;
			
			imgdepth.convertTo(imgDisparity8U1, CV_8UC1, 1.0);
    
			//cout << "minvalue:" << min1 <<"  maxvalue:" << max1 << endl;
			
			//cout << "disparity" << "cols:" << imgDisparity8U.cols << " rows:" << imgDisparity8U.rows <<  " dims: " <<  imgDisparity8U.dims <<  " type: " <<  type2str(imgDisparity8U.type()) <<  " step: " <<  imgDisparity8U.step << endl ;
			//cout << "image" <<"cols:" << ground.cols << " rows:" << ground.rows << " dims: " <<  ground.dims <<  " type: " <<  type2str(ground.type()) << " step: " <<  ground.step << endl ;
		    //cv::imshow("grd1", ground);

			vp.ComputeVdisp(ground , imgDisparity8U1 , 100);
			
	       // cv::imshow("Disp_opencv", imgDisparity8U);
	       // cv::imshow("Disp_calc", imgDisparity8U1);
            //cv::imshow("vDisp", vhist);
            //cv::imshow("grd", ground);
            //cv::imshow("Edges", edges);
			
			
			
			
            vhist = cv::Mat::zeros(new_height, MAX_DISP, CV_32F);
            // Handle key event
            key = cv::waitKey(10);
        }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}


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
