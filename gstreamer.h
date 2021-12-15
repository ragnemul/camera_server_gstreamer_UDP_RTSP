/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   gstreamer.h
 * Author: root
 *
 * Created on March 4, 2017, 12:09 PM
 */
#include "camera_manager.h"

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <nice/agent.h>
#include <nice/address.h>

#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //for exit(0);
#include <sys/socket.h>
#include <errno.h> //For errno - the error number
#include <netdb.h> //hostent
#include <arpa/inet.h>



#ifndef GSTREAMER_H
#define GSTREAMER_H


#define IMG_WIDTH_GST           320 // 320 768
#define IMG_HEIGHT_GST          240 // 240 576

#define IMG_WIDTH           640 // 320 768
#define IMG_HEIGHT          480 // 240 576
#define IMG_JPEG_QUALITY    85  // JPEG image quality percentage
#define MAX_FRAMES_BUFFER   50  // Maximum images buffer size



using namespace cv;
using namespace std;

class Camera_Manager;

class gstreamer {
public:
    gstreamer(Camera_Manager *, string, unsigned short int);
    gstreamer(string, int);
    gstreamer(const gstreamer& orig);
    virtual ~gstreamer();

    static void * start_UDP_gstreaming(void *c);
    void *change_frame_size(gint width, gint height);
    void *change_bitrate(gint bitrate);
    void stop_gstreaming(void *c);
    static void * start_RTSP_gstreaming(void *c);


    string ip;
    int port;

private:
    VideoCapture capture;
    pthread_mutex_t mutex;
    pthread_attr_t attr;
    pthread_t callThd_capture_frames;
    pthread_t callThd_process_frames;
    vector<Mat> frames_buffer;
    void *status;

    struct _App {
        Camera_Manager *camera_manager_;
        GstRTSPServer *server;
        GstRTSPMountPoints *mounts;
        GstRTSPMediaFactory *factory;
        guint gst_server_id;
        GstClockTime timestamp;

        GMainLoop *loop;
        GstElement *pipeline;
        GstElement *appsrc;
        GstElement *videoconvert;
        GstElement *videoscale;
        GstElement *queue1;
        GstElement *videoenc;
        GstElement *queue2;
        GstElement *rtppay;
        GstElement *sink;
        GstElement *filter;
        GstElement *parse;
        GstCaps *caps_scale;
        GstCaps *caps_appsrc;
        GstBus *bus;
        
        guint bus_watch_id;
    };

    typedef struct _App App;

    App s_app;
    App *app = &s_app;

    typedef struct {
        GstClockTime timestamp;
    } MyContext;


    static void transform_img(Mat src, Mat &dst);



    static void cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data);
    static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data);
    static void rtsp_need_data(GstElement * appsrc, guint unused, gpointer user_data);
    static void media_configure(GstRTSPMediaFactory * factory,
            GstRTSPMedia * media, gpointer user_data);

 
};

#endif /* GSTREAMER_H */

