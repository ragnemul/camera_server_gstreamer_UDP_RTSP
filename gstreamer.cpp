/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   gstreamer.cpp
 * Author: root
 * 
 * Created on March 4, 2017, 12:09 PM
 * 
 * Sender:
 
 gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! videoscale name=qos-scaler ! capsfilter name=qos-name caps=video/x-raw,width=1024,height=860,framerate=15/1  ! queue ! x264enc byte-stream=true bitrate=300 speed-preset=superfast subme=6 tune=zerolatency ! queue ! rtph264pay ! udpsink host=127.0.0.1 port=5200
  
 * Receiver: 
  
 gst-launch-1.0 -v udpsrc address=127.0.0.1 port=5200  !  application/x-rtp !  rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! video/x-raw, width=640,height=480,framerate=15/1 ! clockoverlay halignment=right valignment=bottom time-format="%Y/%m/%d %H:%M:%S" shaded-background=true !  autovideosink
 */

#include "gstreamer.h"
#include <boost/format.hpp>
#include <gio/gnetworking.h>

GST_DEBUG_CATEGORY(appsrc_pipeline_debug);
#define GST_CAT_DEFAULT appsrc_pipeline_debug


#define MAX_FRAMES_BUFFER_GST   10  // Maximum images buffer size



gstreamer::gstreamer(Camera_Manager* camera_manager, string ip, unsigned short int port) {
    app -> camera_manager_ = camera_manager;
    this -> ip = ip;
    this -> port = port;
}


gstreamer::gstreamer(string ip, int port) {
    this -> ip = ip;
    this -> port = port;
}


gstreamer::gstreamer(const gstreamer& orig) {
}


gstreamer::~gstreamer() {
    cout << "deleting GStreaming instance!!!!" << endl;
}


void gstreamer::transform_img(Mat src, Mat &dst) {
    Mat gray;
    Mat bin;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    cv::putText(gray, "SmartCow " , cv::Point(10, 200), FONT_HERSHEY_SCRIPT_SIMPLEX, 1, Scalar(255, 255, 255), 2);
    
    threshold(gray, bin, 100, 255, THRESH_BINARY);
            
    cvtColor(bin, dst, COLOR_GRAY2BGR);
}


void gstreamer::cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data) {
    static GstClockTime timestamp = 0;
    static GstBuffer *buffer;
    guint size, height, width, channels;
    GstFlowReturn ret;
    Mat img, frame;
    static guchar *data1;
    static GstMapInfo map;
    App* data = static_cast<App*> (user_data);

#ifdef TBB
    data -> camera_manager_ -> queue.try_pop(frame);
#else 
    data -> camera_manager_ -> queue.pop(frame);
#endif
    
    // transform_img(frame,img);
    img = frame;
        
    
    // img = imread("/usr/local/src/opencv-3.3.1/samples/data/aero1.jpg", CV_LOAD_IMAGE_COLOR);
    // std::cout << "Need data!\n" << std::endl;

    height = img.rows;
    width = img.cols;
    channels = img.channels();
    data1 = (guchar *) img.data;
    size = height * width * channels;

    buffer = gst_buffer_new_allocate(NULL, size, NULL);
    if (gst_buffer_map(buffer, &map, (GstMapFlags) GST_MAP_WRITE)) {
        memcpy((guchar *) map.data, data1, gst_buffer_get_size(buffer));
        gst_buffer_unmap(buffer, &map);
    } else g_print("OPS! ERROR.");

    GST_BUFFER_PTS(buffer) = data -> timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 15);
    data -> timestamp += GST_BUFFER_DURATION(buffer);


    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);

    gst_buffer_unref(buffer);
    // img.release();

    if (ret != GST_FLOW_OK) {
        /* something wrong, stop pushing */
        g_main_loop_quit(data -> loop);
    }
}

gboolean gstreamer::bus_call(GstBus *bus, GstMessage *msg, gpointer data) {

    GST_DEBUG("got message %s",
            gst_message_type_get_name(GST_MESSAGE_TYPE(msg)));

    GMainLoop *loop = (GMainLoop *) data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_QOS:
            g_print("QoS message received on the bus\n");
            cout << "QoS message! " << endl;
            break;

        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            g_main_loop_quit(loop);
            break;

        case GST_MESSAGE_ERROR:
        {
            gchar *debug;
            GError *error;
            gst_message_parse_error(msg, &error, &debug);
            g_printerr("ERROR from element %s: %s\n",
                    GST_OBJECT_NAME(msg->src), error->message);
            g_printerr("Debugging info: %s\n", (debug) ? debug : "none");
            g_free(debug);
            g_error_free(error);
            g_main_loop_quit(loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

void *gstreamer::change_frame_size(gint width, gint height) {
    app -> filter = gst_bin_get_by_name(GST_BIN(app -> pipeline), "qos-filter");

    app -> caps_scale = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            "framerate", GST_TYPE_FRACTION, 15, 1,
            "format", G_TYPE_STRING, "I420",
            NULL);
    g_object_set(G_OBJECT(app->filter), "caps", app -> caps_scale, NULL);
    gst_caps_unref(app -> caps_scale);
    gst_object_unref(app -> filter);
}

void *gstreamer::change_bitrate(gint bitrate) {
    g_object_set(G_OBJECT(app->videoenc), "bitrate", bitrate, NULL);
}

void gstreamer::stop_gstreaming(void *c) {
    gstreamer *g = ((gstreamer *) c);

    cout << "stopping gstreamer" << endl;
    g_main_loop_quit(g -> app -> loop);
}



/*
 gst-launch-1.0 -v udpsrc address=127.0.0.1 port=5200  !  application/x-rtp !  rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! video/x-raw, width=640,height=480,framerate=15/1 ! clockoverlay halignment=right valignment=bottom time-format="%Y/%m/%d %H:%M:%S" shaded-background=true !  autovideosink
 */
void *gstreamer::start_UDP_gstreaming(void *c) {
    gstreamer *g = ((gstreamer *) c);
    g->app->timestamp = 0;

    cout << "Starting UDP gstreamer server to: " << g -> ip << ":" << g -> port << endl;
    /* init GStreamer */

    GST_DEBUG_CATEGORY_INIT(appsrc_pipeline_debug, "appsrc-pipeline", 0,
            "appsrc pipeline example");

    g -> app -> loop = g_main_loop_new(NULL, FALSE);
    

    /* setup pipeline */
    g -> app -> pipeline = gst_pipeline_new("pipeline");
    if (!g -> app -> pipeline) {
        g_print("Error creating Pipeline, exiting...");
    }

    // g -> app -> appsrc = gst_element_factory_make("v4l2src", "source");
    g -> app -> appsrc = gst_element_factory_make("appsrc", "source");
    if (!g -> app -> appsrc) {
        g_print("Error creating source element, exiting...");
    }
    g -> app -> caps_appsrc = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, IMG_WIDTH,
            "height", G_TYPE_INT, IMG_HEIGHT,
            "framerate", GST_TYPE_FRACTION, 15, 1,
            "format", G_TYPE_STRING, "BGR",
            NULL);
    g_object_set(G_OBJECT(g -> app -> appsrc), "caps", g -> app -> caps_appsrc, NULL);
    gst_caps_unref(g -> app -> caps_appsrc);

    GST_DEBUG("got appsrc %p", g -> app->appsrc);


    g -> app -> videoenc = gst_element_factory_make("x264enc", "videoenc");
    if (!g -> app -> videoenc) {
        std::cout << "Error creating encoder, exiting...";
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->videoenc), "name", "videoenc");
    g_object_set(G_OBJECT(g -> app->videoenc), "bitrate", 200, NULL); // 75
    g_object_set(G_OBJECT(g -> app->videoenc), "byte-stream", true, NULL);
    gst_util_set_object_arg(G_OBJECT(g -> app->videoenc), "speed-preset", "superfast");
    gst_util_set_object_arg(G_OBJECT(g -> app->videoenc), "tune", "zerolatency");
    g_object_set(G_OBJECT(g -> app->videoenc), "subme", 6, NULL);

    g -> app -> parse = gst_element_factory_make("h264parse", "parse");
    if (!g -> app -> videoenc) {
        std::cout << "Error creating encoder, exiting...";
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->parse), "name", "parse");

    g -> app -> rtppay = gst_element_factory_make("rtph264pay", "rtppay");
    if (!g -> app -> rtppay) {
        std::cout << "Error creating rtph264pay, exiting...";
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->rtppay), "name", "rtppay");








    g -> app -> videoconvert = gst_element_factory_make("videoconvert", "videoconvert");
    if (!g -> app -> videoconvert) {
        g_print("Error creating videoconvert element, exiting...");
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->videoconvert), "name", "videoconvert");

    g -> app -> videoscale = gst_element_factory_make("videoscale", "videoscale");
    if (!g -> app -> videoscale) {
        g_print("Error creating videoscale element, exiting...");
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->videoscale), "name", "qos-scaler");


    //creazione elemento queue:
    g -> app-> queue1 = gst_element_factory_make("queue", "queue1");
    if (!g -> app->queue1) {
        g_print("Error creating queue element, exiting...");
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->queue1), "name", "queue1");

    //creazione elemento queue:
    g -> app-> queue2 = gst_element_factory_make("queue", "queue2");
    if (!g -> app->queue2) {
        g_print("Error creating queue element, exiting...");
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->queue2), "name", "queue2");


    // app -> sink = gst_element_factory_make("autovideosink", "sink");
    g -> app -> sink = gst_element_factory_make("udpsink", "sink");
    if (!g -> app -> sink) {
        std::cout << "Error creating sink, exiting...";
    }
    // g_object_set(G_OBJECT(g -> app->sink), "host", "127.0.0.1", NULL); 
    // g_object_set(G_OBJECT(g -> app->sink), "port", 5200, NULL);
    g_object_set(G_OBJECT(g -> app->sink), "host", strcpy((char*) malloc(g -> ip.length() + 1), g -> ip.c_str()), NULL);
    g_object_set(G_OBJECT(g -> app->sink), "port", gint(g -> port), NULL);
    gst_util_set_object_arg(G_OBJECT(g -> app->sink), "name", "sink");


    g -> app -> filter = gst_element_factory_make("capsfilter", "filter");
    g_assert(g -> app->filter != NULL); /* should always exist */
    if (!g -> app -> filter) {
        std::cout << "Error creating filter, exiting...";
    }
    gst_util_set_object_arg(G_OBJECT(g -> app->filter), "name", "qos-filter");

    g -> app -> caps_scale = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, IMG_WIDTH_GST,
            "height", G_TYPE_INT, IMG_HEIGHT_GST,
            "framerate", GST_TYPE_FRACTION, 15, 1,
            "format", G_TYPE_STRING, "I420",
            NULL);
    g_object_set(G_OBJECT(g -> app->filter), "caps", g -> app -> caps_scale, NULL);
    gst_caps_unref(g -> app -> caps_scale);

    // Bus association to pipeline
    g -> app -> bus = gst_pipeline_get_bus(GST_PIPELINE(g -> app -> pipeline));
    // event handle to bus association

    g -> app -> bus_watch_id = gst_bus_add_watch(g -> app -> bus, bus_call, g -> app -> loop);
    gst_object_unref(g -> app -> bus);


    
    gst_bin_add_many(GST_BIN(g -> app -> pipeline),
            g -> app -> appsrc,
            g -> app -> videoconvert,
            g -> app -> videoscale,
            g -> app -> filter,
            g -> app -> videoenc,
            g -> app -> rtppay,
            g -> app -> sink, NULL);
    g_print("Added all the Elements into the pipeline\n");

    /*
        cout << "source -> videoconvert: " << gst_element_link(app -> appsrc, app -> videoconvert) << endl;
        cout << "videconvert -> videoscale: " << gst_element_link(app -> videoconvert, app -> videoscale) << endl;
        cout << "videoscale -> filter: " << gst_element_link(app -> videoscale, app -> filter) << endl;
        cout << "filter -> videoenc: " << gst_element_link(app -> filter, app -> videoenc) << endl;
        cout << "videoenc -> rtppay: " << gst_element_link(app -> videoenc, app -> rtppay) << endl;
        cout << "rtppay -> sink: " << gst_element_link(app -> rtppay, app -> sink) << endl;
     */

    gboolean linked = gst_element_link_many(
            g -> app -> appsrc,
            g -> app -> videoconvert,
            g -> app -> videoscale,
            g -> app -> filter,
            g -> app -> videoenc,
            g -> app -> rtppay,
            g -> app -> sink, NULL);
    if (linked)
        g_print("Linked all the Elements into the pipeline\n");
    else
        g_print("Error linking elements into pipelile. Exiting...");

    /* receiver:
    gst-launch-1.0 -v udpsrc address=127.0.0.1 port = 5200 !application/x rtp !
            rtph264depay !avdec_h264 ! videoconvert !videoscale !
            video/x-raw, width=640, height=480, framerate=15/1 !
            clockoverlay halignment=right valignment=bottom time-format="%Y/%m/%d %H:%M:%S" shaded-background=true !autovideosink
     */


    /* setup app -> appsrc */

    g_object_set(G_OBJECT(g -> app -> appsrc),
            "stream-type", 0,
            "format", GST_FORMAT_TIME, NULL);

    g_signal_connect(g -> app -> appsrc, "need-data", G_CALLBACK(cb_need_data), g -> app);

    /* play */
    GstStateChangeReturn ret = gst_element_set_state(g -> app -> pipeline, GST_STATE_PLAYING);
    g_print("Setting to READY\n");
    
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the ready state.\n");
        gst_object_unref(g -> app -> pipeline);
        goto end; // new
    }
    
    ret = gst_element_set_state(g -> app -> pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(g -> app -> pipeline);
        goto end; // new
    }

    g_main_loop_run(g -> app -> loop);
    g_print("Returned, stopping playback\n");

    /* clean up */
    gst_pad_push_event(gst_element_get_static_pad(g -> app -> appsrc, "source"), gst_event_new_eos()); // new
    gst_element_unlink_many(g -> app -> appsrc, g -> app -> videoconvert, g -> app -> videoscale,
            g -> app -> filter, g -> app -> videoenc, g -> app -> rtppay, g -> app -> sink, NULL);

    gst_element_set_state(g -> app -> pipeline, GST_STATE_NULL);

    g_print("Deleting pipeline\n");
    gst_bin_remove_many(GST_BIN(g -> app -> pipeline), g -> app -> appsrc, g -> app -> videoconvert, g -> app -> videoscale,
            g -> app -> filter, g -> app -> videoenc, g -> app -> rtppay, g -> app -> sink, NULL);
    
end:
        
    g_source_remove(g -> app -> bus_watch_id); // new    

    gst_object_unref(GST_OBJECT(g -> app -> pipeline));
    g_main_loop_unref(g -> app -> loop);
    

    return 0;
}



void gstreamer::rtsp_need_data(GstElement * appsrc, guint unused, gpointer user_data) {
    GstBuffer *buffer;
    GstFlowReturn ret;
    guint size, height, width, channels;
    Mat img;
    guchar *data1;
    GstMapInfo map;
    App* data = static_cast<App*> (user_data);

    // get the frame
    data -> camera_manager_ -> queue.pop(img);
    //img = imread("/usr/local/src/opencv-3.3.1/samples/data/aero1.jpg", CV_LOAD_IMAGE_COLOR);
    //std::cout << "Need data!\n" << std::endl;

    height = img.rows;
    width = img.cols;
    channels = img.channels();
    data1 = (guchar *) img.data;
    size = height * width*channels;

    buffer = gst_buffer_new_allocate(NULL, size, NULL);
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy((guchar *) map.data, data1, gst_buffer_get_size(buffer));
    gst_buffer_unmap(buffer, &map);

    /* increment the timestamp every 1/15 second */
    GST_BUFFER_PTS(buffer) = data -> timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 15);
    data -> timestamp += GST_BUFFER_DURATION(buffer);

    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
}

/* called when a new media pipeline is constructed. We can query the
 * pipeline and configure our appsrc */
void gstreamer::media_configure(GstRTSPMediaFactory * factory, GstRTSPMedia * media,
        gpointer user_data) {
    App* app = static_cast<App*> (user_data);

    /* get the element used for providing the streams of the media */
    app -> pipeline = gst_rtsp_media_get_element(media);

    /* get our appsrc, we named it 'mysrc' with the name property */
    app -> appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(app -> pipeline), "mysrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg(G_OBJECT(app -> appsrc), "format", "time");
    /* configure the caps of the video */
    g_object_set(G_OBJECT(app -> appsrc), "caps",
            gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, IMG_WIDTH, // 384
            "height", G_TYPE_INT, IMG_HEIGHT, // 288
            "framerate", GST_TYPE_FRACTION, 15, 1, NULL), NULL);

    app -> timestamp = 0;

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect(app -> appsrc, "need-data", (GCallback) rtsp_need_data, app);
    gst_object_unref(app -> appsrc);
    // gst_object_unref(app -> pipeline);

    cout << "media configured!" << std::endl;
}


/*
 
 gst-launch-1.0 rtspsrc location="rtsp://192.168.1.55:8554/test"  ! rtph264depay  ! avdec_h264 ! videoconvert ! autovideosink
 
 */
void *gstreamer::start_RTSP_gstreaming(void *c) {
    gstreamer *g = ((gstreamer *) c);

    //    gst_init(NULL, NULL);

    g -> app -> loop = g_main_loop_new(NULL, FALSE);

    /* create a server instance */
    g -> app -> server = gst_rtsp_server_new();
    //    gst_rtsp_server_set_address(g -> app -> server, "192.168.42.47");
    gst_rtsp_server_set_address(g -> app -> server, g -> ip.c_str());

    /* get the mount points for this server, every server has a default object
     * that be used to map uri mount points to media factories */
    g -> app -> mounts = gst_rtsp_server_get_mount_points(g -> app -> server);

    /* make a media factory for a test stream. The default media factory can use
     * gst-launch syntax to create pipelines.
     * any launch line works as long as it contains elements named pay%d. Each
     * element with pay%d names will be a stream */
    g -> app -> factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_shared(g -> app -> factory, TRUE);

    gchar *pipe_desc =
            g_strdup_printf("( appsrc name=mysrc ! videoconvert ! "
            "videoscale name=videoscale ! "
            "capsfilter name=qos-filter caps=video/x-raw,width=320,height=240,framerate=15/1,format=(string)I420 ! "
            "queue name=queue1 ! "
            "x264enc name=videoenc byte-stream=true bitrate=100 speed-preset=superfast subme=6 tune=zerolatency  ! "
            "queue name=queue2 ! "
            "rtph264pay name=pay0 pt=96 )");
    // for viewer connected through 4G bitrate=150 and 320x240 works fine

    gst_rtsp_media_factory_set_launch(g -> app -> factory, pipe_desc);
    g_free(pipe_desc);


    /* notify when our media is ready, This is called whenever someone asks for
     * the media and a new pipeline with our appsrc is created */
    g_signal_connect(g -> app -> factory, "media-configure",
            (GCallback) media_configure, g -> app);

    /* attach the test factory to the /test url */
    gst_rtsp_mount_points_add_factory(g -> app -> mounts,
            "/test", g -> app -> factory);


    /* don't need the ref to the mounts anymore */
    g_object_unref(g -> app -> mounts);

    /* attach the server to the default maincontext */
    g -> app -> gst_server_id = gst_rtsp_server_attach(g -> app -> server, NULL);

    /* start serving */
    std::cout << "\nstream ready at rtsp://" << g -> ip.c_str() << ":8554/test\n" << std::endl;
    g_main_loop_run(g -> app -> loop);
}





