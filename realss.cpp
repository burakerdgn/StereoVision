// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_processing.hpp>
#include <librealsense2/rs_frame.hpp>


//Open-CV
#include <opencv2/opencv.hpp> // Include OpenCV API
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdint.h>
#include <unistd.h>

//Add vdisp lab
#include "vdisp.h"

#include <array>

#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;

#define MAX_DISP 100


int main(int argc, char * argv[]) try
{
	Vdisp vp;
	
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
    

    const auto window_name = "Depth_color";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    const auto window_name1 = "Color";
    namedWindow(window_name1, WINDOW_AUTOSIZE);


	//char key = ' ';
	//while (key != 'q')
    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    //while (true)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame depth2 = data.get_depth_frame();
        rs2::frame color_frame = data.get_color_frame();
        rs2::pipeline_profile pipeline_profile;
        
        // Transform Disparity Frame from Depth Frame
		rs2::disparity_transform disparity_transform( true );
		rs2::frame disparity_frame = disparity_transform.process( depth2 );
		
		// Retrive BaseLine
		float baseline = disparity_frame.as<rs2::disparity_frame>().get_baseline();

		// Retrive Frame Size
		uint32_t disparity_width = disparity_frame.as<rs2::video_frame>().get_width();
		uint32_t disparity_height = disparity_frame.as<rs2::video_frame>().get_height();
		

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        
        const int color_width = color_frame.as<rs2::video_frame>().get_width();
		const int color_height = color_frame.as<rs2::video_frame>().get_height();

		Mat color(Size(color_width, color_height), CV_8UC3, (void*)color_frame.get_data());
	
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
        
        Mat image2(Size(w, h), CV_16SC1, (void*)depth2.get_data(), Mat::AUTO_STEP);
        
        Mat scale_mat;
		image2.convertTo( scale_mat, CV_8U, 255.0 / 10000.0, 0 ); // 0-10000 -> 255(white)-0(black)
		
		Mat disparity_mat;
		disparity_mat = cv::Mat( disparity_height, disparity_width, CV_32FC1, const_cast<void*>( disparity_frame.get_data() ) );
        

		Mat imgDisparity8U = Mat(disparity_height, disparity_width, CV_8UC1);
		
		disparity_mat.convertTo(imgDisparity8U, CV_8UC1, 1.0/16.0);
		
		  
		vp.ComputeVdisp(scale_mat, imgDisparity8U,MAX_DISP);
		//imshow("vDisp", vhist);
		//imshow("Edges", edges);
		imshow(window_name, depth_image);
        imshow(window_name1, color);
        //imshow("depth2",scale_mat);
        //imshow( "Disparity", imgDisparity8U );
				
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


