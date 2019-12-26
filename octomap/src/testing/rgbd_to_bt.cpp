// File downloaded from https://gist.github.com/mike168m/6dd4eb42b2ec906e064d


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <string>
#include <assert.h>

#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include "testing.h"

using namespace std;
using namespace octomap;
using namespace octomath;

#define IMG_WIDTH 1280
#define IMG_HEIGHT 720

int main(int argc, char *argv[]) {
    // 1.  Open the device
    int fd; // A file descriptor to the video device
    fd = open("/dev/video0", O_RDWR);
    if(fd < 0){
        perror("Failed to open device, OPEN");
        return 1;
    }


    // 2. Ask the device if it can capture frames
    v4l2_capability capability;
    if(ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0){
        // something went wrong... exit
        perror("Failed to get device capabilities, VIDIOC_QUERYCAP");
        return 1;
    }


    // 3. Set Image format
    v4l2_format imageFormat;
    imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    imageFormat.fmt.pix.width = 1280;
    imageFormat.fmt.pix.height = 720;
    //imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    imageFormat.fmt.pix.pixelformat = v4l2_fourcc('Y', 'U', 'Y', 'V');
    imageFormat.fmt.pix.field = V4L2_FIELD_NONE;
    // tell the device you are using this format
    if(ioctl(fd, VIDIOC_S_FMT, &imageFormat) < 0){
        perror("Device could not set format, VIDIOC_S_FMT");
        return 1;
    }


    // 4. Request Buffers from the device
    v4l2_requestbuffers requestBuffer = {0};
    requestBuffer.count = 1; // one request buffer
    requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // request a buffer wich we an use for capturing frames
    // requestBuffer.memory = V4L2_MEMORY_MMAP;
    requestBuffer.memory = V4L2_MEMORY_USERPTR;

    if(ioctl(fd, VIDIOC_REQBUFS, &requestBuffer) < 0){
        perror("Could not request buffer from device, VIDIOC_REQBUFS");
        return 1;
    }

    v4l2_buffer queryBuffer = {0};
    queryBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queryBuffer.memory = V4L2_MEMORY_USERPTR;
    //queryBuffer.index = 0;
    queryBuffer.index = 0;
    if(ioctl(fd, VIDIOC_QUERYBUF, &queryBuffer) < 0){
        perror("Device did not return the buffer information, VIDIOC_QUERYBUF");
        return 1;
    }

    unsigned int length = 1280 * 720 * 4;
    unsigned char *bigBuffer = (unsigned char *) malloc (length);
    assert (NULL != bigBuffer);
    memset (bigBuffer, 0, length);

    v4l2_buffer bufferinfo;
    memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_USERPTR;
    //bufferinfo.index = 0;
    bufferinfo.index = 0;
    bufferinfo.m.userptr = (unsigned long) bigBuffer;
    bufferinfo.length = length;

    // Activate streaming
    int type = bufferinfo.type;
    if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
        perror("Could not start streaming, VIDIOC_STREAMON");
        return 1;
    }

    //Intel RGBD calibration parameters
    float fx = 500.0 , cx = 640.00, cy = 360.00;
    //float baseline = 0.12;
    //float units = 1;
    float nearest_pixel = 0.0, farthest_pixel = 5.0;
    Pointcloud p;
    OcTree tree (0.02);  
    point3d origin (0,0,0);
    cout << "Reconstructing 3d map " << origin << " ..." << endl;
/***************************** Begin looping here *********************/
  for (int z = 0; z < 2; z++) {
    // Queue the buffer
    if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
        perror("Could not queue buffer, VIDIOC_QBUF");
        return 1;
    }

    // Dequeue the buffer
    if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0){
        perror("Could not dequeue the buffer, VIDIOC_DQBUF");
        return 1;
    }
    // Frames get written after dequeuing the buffer

    cout << "Buffer has: " << bufferinfo.bytesused << " bytes of data" << endl;
    printf ("flags=[0x%08x]\n", bufferinfo.flags);

    char buf[1024];
    sprintf (buf, "raw_data%02d.bin", z);
    FILE *pFout = fopen (buf, "w");
    // size_t fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
    fwrite (bigBuffer, length, 1, pFout);
    fclose (pFout);

    usleep (10000);
  }
 
    short * DepthData ;
    DepthData = (short *)bigBuffer;
    for(int i = 0; i < IMG_HEIGHT; i++) {
        for(int j = 0; j < IMG_WIDTH; j++) {
            float d = (float)DepthData[i*IMG_WIDTH + j]/1000.0;
            if ((d)>nearest_pixel&&((d))<farthest_pixel) {
                float x = float((i-cy)*d/fx), y = float((j-cx)*d/fx);
                //cout<<x<<" "<<y<<" "<<d<<endl;
                point3d point_on_surface (x , y, d);
                p.push_back(point_on_surface);
            }
        }
    }
  tree.insertPointCloud(p, origin);

  //munmap(ptr, res_mem_length);
  //close(fd);
  cout << "Writing to custom_image.bt..." << endl;
  EXPECT_TRUE(tree.writeBinary("custom_image.bt"));

  std::cout << "Test successful\n";
/******************************** end looping here **********************/

    // end streaming
    if(ioctl(fd, VIDIOC_STREAMOFF, &type) < 0){
        perror("Could not end streaming, VIDIOC_STREAMOFF");
        return 1;
    }

    close(fd);
    return 0;
}


