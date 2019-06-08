
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

    OcTree tree (0.02);  
	string image_name[3] ;
	bool invert = true;
    if( argc < 2)
    {
       cout<<"Usign inbuilt image ";
       //image_name = "rs_fan_bon_depth_intel.png";
       image_name[0] = "1_Depth.png";
       image_name[1] = "2_Depth.png";
       image_name[2] = "3_Depth.png";

    }
    else
    {
    	image_name[0] = argv[1]; 
    }
    if(argc==3)
    	if(strcmp(argv[2],"true")==0) invert = true; 

    int image_numbers = 3;

    for(int t=0;t<image_numbers;t++)
    {
    	point3d origin (0 + t*0.013,0,0);
    	Mat image;
    
	    image = imread(image_name[t], 0);   // Read the file

	    if(! image.data )                              // Check for invalid input
	    {
	        cout <<  "Could not open or find the image" << std::endl ;
	        return -1;
	    }
	    std::cout << image.rows <<" rows "<<image.cols<<" columns "<<int(image.at<uchar>(120,120)); 
	    float fx = 400.8;                    //942.8
	    float baseline = 49.9;              //51 
	    float units = 100;
	    Mat depth(image.rows, image.cols, CV_32F);
	    for(int i = 0; i < image.rows; i++)
	    {
	        for(int j = 0; j < image.cols; j++)
	        {
	        	if(image.at<uint8_t>(i,j)>3 && image.at<uint8_t>(i,j) <252)
	        	{
		        	if(invert)
		            	depth.at<float>(i,j)= float((fx * baseline) / (units *(255-image.at<uint8_t>(i,j))));
		        	else
		        		depth.at<float>(i,j)= float((fx * baseline) / (units *(image.at<uint8_t>(i,j))));
	        	}

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

	  cout << "Reconstructing 3d map " << origin << " ..." << endl;

	  Pointcloud p;
	  for(int i = 0; i < image.rows; i++)
	    {
	        for(int j = 0; j < image.cols; j++)
	        {
	        	float d = depth.at<float>(i,j);
	        	point3d point_on_surface (float((i-image.rows/2)*d/fx), float((j-image.cols/2)*d/fx),d );
	          //if(d > 1 and d<30)
	          p.push_back(point_on_surface);
	        }
	    }

	  tree.insertPointCloud(p, origin);
    }
    


  cout << "Writing to custom_image.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("bt_folder/custom_image.bt"));

  std::cout << "Test successful\n";

//------------------------------------------





  return 0;
}
