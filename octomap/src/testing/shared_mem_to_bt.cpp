// This program takes input of the physical address where the disparity image is stored 
// in RAM and creates a point cloud based on the zedcameras calibration parameters.
// The final point cloud is written to a .bt file.


#include <stdio.h>
#include <string>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"
#include <iostream>
#include <fstream> 
#include <sstream>
//#include <boost/lexical_cast.hpp>// for lexical_cast()

#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

using namespace std;
using namespace octomap;
using namespace octomath;

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

int main(int argc, char** argv) {
  cout << argv[0] << " : Entering" << endl;

  // 1st Argument image address 
  ofstream fout; 
  string line;
  fout.open("3d_points.txt");
  unsigned int Image_location;
    if( argc < 2)
    {
       cout<<"Command: ./shared_mem_bt (Argument Disparity Image physical address_";
       return -1;
    }
    else
    {
      //Image_location = boost::lexical_cast<unsigned int >(argv[1]);
      Image_location = std::atoi( argv[1] );
    }

    int fd = open ("/dev/mem", O_RDWR);
    if (fd < 0) {
        printf ("ERROR : failed to open /dev/mem\n");
        return -1;
    }
    unsigned int res_mem_low_addr = Image_location;
    unsigned int res_mem_length   = 0x1000000; //32MB
    cout <<  res_mem_low_addr;

    unsigned int page_size   =  sysconf(_SC_PAGESIZE);
    unsigned int page_addr   = (res_mem_low_addr & (~(page_size-1)));
    unsigned int page_offset =  res_mem_low_addr - page_addr;
    unsigned int rounded_length = page_size * ((res_mem_length + page_offset + (page_size - 1)) / page_size);

    // void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
    unsigned char *ptr = (unsigned char *) mmap (NULL, rounded_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);
    if (MAP_FAILED == ptr) {
      perror ("mmap failed : ");
      bool flag = true;
      while (flag) {
        sleep (1);
      }
    }
    ptr += page_offset;

    printf ("INFO : /dev/mem mapped at [0x%08x]\n",  ptr);

    //Calibration parameters of zed camera
    float fx = 350.5 , cx = 348.233, cy = 193.477;
    float baseline = 0.12;
    float units = 1;
    int nearest_pixel = 10, farthest_pixel = 85; 

  OcTree tree (0.02);  
  point3d origin (0,0,0);
  cout << "Reconstructing 3d map " << origin << " ..." << endl;

  Pointcloud p;
  for(int i = 0; i < IMG_HEIGHT; i++)
  {
    for(int j = 0; j < IMG_WIDTH; j++)
    {
			if (((ptr[i*IMG_WIDTH + j]))>nearest_pixel&&((ptr[i*IMG_WIDTH + j]))<farthest_pixel)
      {
            	float d = float((fx * baseline) / (units * (ptr[i*IMG_WIDTH + j])));
		          float x = float((i-cy)*d/fx), y = float((j-cx)*d/fx);
		          point3d point_on_surface (x , y, d);
              p.push_back(point_on_surface);
              #if 0
		          std::ostringstream a,b,c;
		          a << x; b << y; c << d;
		          line = a.str() + "," + b.str() + "," + c.str(); 
              fout << line << endl;
              #endif
      } 
    }
  }
  tree.insertPointCloud(p, origin);

  //munmap(ptr, res_mem_length);
  //close(fd);
  fout.close();
  cout << "Writing to custom_image.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("custom_image.bt"));

  std::cout << "Test successful\n";

//------------------------------------------


  cout << argv[0] << " : Exiting." << endl;


  return 0;
}
