/* 
 * File:   camera_manager.h
 * Author: root
 *
 * Created on February 9, 2016, 6:49 AM
 */

#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include "gstreamer.h"

#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <string.h>
#include <stdint.h>
#include <libv4l2.h>
#include <vector>

#include <boost/asio.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


#ifdef TBB
#include <tbb/concurrent_queue.h>
#else
#include <boost/lockfree/spsc_queue.hpp>
#endif


#include "gstreamer.h"


// camera capabilities (USB)
// #udevadm trigger
// #lsusb
// #lsusb -s 001:002 -v | egrep "Width|Height"
// 1280x720 mac book pro camera exported to parallels
// logitech supports 320x240, ..., 960x720, 1600x720
// picam:   3280 x 2464

/*
#	Resolution	Aspect Ratio	Framerates	Video	Image	FoV	Binning
1	1920x1080	16:9            0.1-30fps	x	 	Partial	None
2	3280x2464	4:3             0.1-15fps	x	x	Full	None
3	3280x2464	4:3             0.1-15fps	x	x	Full	None
4	1640x1232	4:3             0.1-40fps	x	 	Full	2x2
5	1640x922	16:9            0.1-40fps	x	 	Full	2x2
6	1280x720	16:9            40-90fps	x	 	Partial	2x2
7	640x480         4:3             40-90fps	x	 	Partial	2x2 
*/

#define IMG_WIDTH           640 // 320 768
#define IMG_HEIGHT          480 // 240 576
#define IMG_JPEG_QUALITY    85  // JPEG image quality percentage
#define MAX_FRAMES_BUFFER   50  // Maximum images buffer size

using namespace std;
#ifdef TBB
using namespace tbb;
#endif



class Camera_Manager {
public:
    Camera_Manager(string device, string ip, unsigned short int port);
    Camera_Manager(const Camera_Manager& orig);
    
    virtual ~Camera_Manager();
    void cancel_camera_process();
    void start_camera_process();

    static Mat get_frame(void *c);
    static Mat get_front_frame(void *c);
    static bool frames_buffer_has_elements(void *c);
    void ChangeFrameSize(int weight, int height);
    void ChangeBitrate(int bitrate);

    Mat frame;
    string ip_dest;

    pthread_mutex_t mutex;
    pthread_mutex_t mutex_frame;
    
    
    
#ifdef TBB
    concurrent_queue<Mat> queue;
#else
    boost::lockfree::spsc_queue<Mat> queue{100};
#endif


private:  
    
    
    bool send_images;


    pthread_attr_t attr_capture_frames;
    pthread_attr_t attr_process_frames;
    pthread_t callThd_capture_frames;
    pthread_t callThd_process_frames;
    void *status;

    uint8_t * buffer;
    bool camActive;
    bool camera_flag;
    
    unsigned short int port_dest;
    string video_device;
    vector<Mat> frames_buffer;


    long diff(struct timespec start, struct timespec end);
    int xioctl(int fd, int request, void * arg);
    int print_caps(int fd);
    int set_image_format(int fd);
    int init_mmap(int fd);
    static void * capture_frames(void *c);
    static void * process_frames(void *c);

    static void *insert_frame(void *c);


};

#endif /* CAMERA_MANAGER_H */

