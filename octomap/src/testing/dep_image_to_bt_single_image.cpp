
#include <stdio.h>
#include <string.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>



using namespace cv;
using namespace std;
using namespace octomap;
using namespace octomath;



int main(int argc, char** argv) {


	string image_name = "";
	bool invert = false;
    if( argc < 2)
    {
       cout<<"Usign inbuilt image ";
       image_name = "rs_fan_bon_depth_intel.png";
    }
    else
    {
    	image_name = argv[1]; 
    }
    if(argc==3)
    	if(strcmp(argv[2],"true")==0) invert = true; 
    Mat image;
    
    image = imread(image_name, 0);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    std::cout << image.rows <<" rows "<<image.cols<<" columns "<<int(image.at<uchar>(120,120)); 
    float fx = 942.8;
    float baseline = 54.8;
    float units = 100;
    Mat depth(image.rows, image.cols, CV_32F);
    for(int i = 0; i < image.rows; i++)
    {
        for(int j = 0; j < image.cols; j++)
        {
        	if(invert)
            	depth.at<float>(i,j)= float((fx * baseline) / (units *(255-image.at<uint8_t>(i,j))));
        	else
        		depth.at<float>(i,j)= float((fx * baseline) / (units *(image.at<uint8_t>(i,j))));
        }
    }

/*    namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.
    namedWindow( "Display window2", WINDOW_NORMAL );// Create a window for display.
    imshow( "Display window2", depth );                   // Show our image inside it

    waitKey(0);*/                                          // Wait for a keystroke in the window
    


    //For image size 480 x 640
    // real_X = (x-240)*real_depth/focal_length
    // real_Y = (y-320)*real_depth/focal_length

//----------------------------------------------------------
  OcTree tree (0.03);  
  point3d origin (0,0,0);
  cout << "Reconstructing 3d map " << origin << " ..." << endl;

  Pointcloud p;
  for(int i = 0; i < image.rows; i++)
    {
        for(int j = 0; j < image.cols; j++)
        {
        	float d = depth.at<float>(i,j);
        	point3d point_on_surface (float((i-image.rows/2)*d/fx), float((j-image.cols/2)*d/fx),d );
            p.push_back(point_on_surface);
        }
    }
  tree.insertPointCloud(p, origin);


  cout << "Writing to custom_image.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("custom_image.bt"));

  std::cout << "Test successful\n";

//------------------------------------------





  return 0;
}
