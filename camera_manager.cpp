/* 
 * File:   camera_manager.cpp
 * Author: root
 * 
 * Created on February 9, 2016, 6:49 AM
 *  gst-launch-1.0 rtspsrc protocol=4 location="rtsp://192.168.1.55:8554/test" drop-on-latency=true ! rtph264depay ! queue name=queue1  ! avdec_h264 lowres=0 skip-frame=0 max-threads=0 output-corrupt=false ! queue name=queue2  ! videoconvert qos=true ! autovideosink
 */

#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include "camera_manager.h"


#include <vector>


using namespace cv;
using boost::asio::ip::tcp;
using namespace std;

namespace patch {

    template < typename T > std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}

// Starts Camera_Manager sending images throught local network

Camera_Manager::Camera_Manager(string device, string ip, unsigned short int port) {
    camActive = false;

    this->camera_flag = camera_flag;
    this->ip_dest = ip;
    this->port_dest = port;
    this->video_device = device;

    

    pthread_mutex_init(&mutex_frame, NULL);
    pthread_mutex_init(&mutex, NULL);


    pthread_attr_init(&attr_capture_frames);
    pthread_attr_setdetachstate(&attr_capture_frames, PTHREAD_CREATE_JOINABLE);
    pthread_create(&callThd_capture_frames, &attr_capture_frames, Camera_Manager::capture_frames, this);

     
    pthread_attr_init(&attr_process_frames);
    pthread_attr_setdetachstate(&attr_process_frames, PTHREAD_CREATE_JOINABLE);
    pthread_create(&callThd_process_frames, &attr_process_frames, Camera_Manager::process_frames, this);

    send_images = false;
    gstreamer gst(this, this -> ip_dest, this -> port_dest);
    // gst.start_RTSP_gstreaming(&gst);
    gst.start_UDP_gstreaming(&gst);
    
    /*
    gst = new gstreamer(this, this -> ip_dest);
    // g.start_UDP_gstreaming(&g);
    gst.start_RTSP_gstreaming(&gst);

*/
    pthread_join(callThd_capture_frames, &status);
    pthread_join(callThd_process_frames, &status);
}

Camera_Manager::Camera_Manager(const Camera_Manager& orig) {
}

Camera_Manager::~Camera_Manager() {
    if (camera_flag) {
        // waits until threat termination
        pthread_join(callThd_capture_frames, &status);
        pthread_join(callThd_process_frames, &status);
        printf("Thread capture_frames: completed join  having a status  of %ld\n", (long) status);

        pthread_attr_destroy(&attr_capture_frames);
        pthread_mutex_destroy(&mutex_frame);
    }
}

void Camera_Manager::cancel_camera_process() {
    pthread_cancel(callThd_capture_frames);
}

void Camera_Manager::start_camera_process() {
    pthread_attr_init(&attr_capture_frames);
    pthread_attr_setdetachstate(&attr_capture_frames, PTHREAD_CREATE_JOINABLE);
    pthread_create(&callThd_capture_frames, &attr_capture_frames, Camera_Manager::capture_frames, this);
}



void *Camera_Manager::insert_frame(void *c) {
    pthread_mutex_lock(&((Camera_Manager *) c) -> mutex);

    if (((Camera_Manager *) c) -> frames_buffer.size() < MAX_FRAMES_BUFFER) {
        ((Camera_Manager *) c) -> frames_buffer.push_back(((Camera_Manager *) c) -> frame);
    }

    pthread_mutex_unlock(&((Camera_Manager *) c) -> mutex);
}



void *Camera_Manager::capture_frames(void *c) {
    int fd;
    bool error = false;
    string ip;
    string video_device;
    VideoCapture cap;
    bool video_file = false;
    unsigned short int port;

    ip = ((Camera_Manager *) c)->ip_dest;
    port = ((Camera_Manager *) c)->port_dest;

    string device_video_str = "/dev/video";
    video_device = ((Camera_Manager *) c) -> video_device;
    video_file = (video_device.find(device_video_str) != std::string::npos) ? false : true;

    if (!video_file) { // Camera used
        int camera_device = boost::lexical_cast<int>(video_device.substr(device_video_str.length()));
                
        cap.open(camera_device);
        // cap.open("v4l2src device=/dev/video0 ! videoscale ! videorate ! video/x-raw, format=YUY2, width=320, height=240, framerate=15/1 ! videoconvert ! appsink");

        if (!cap.isOpened()) {
            error = true;
            perror("Opening Video Device");
            pthread_mutex_destroy(&((Camera_Manager*) c)->mutex);
        }

        // cap.set(CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
        // cap.set(CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
        
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);

    } else { // video file used
        cap.open(video_device);
        if (!cap.isOpened()) {
            perror("Opening Video File");
            pthread_mutex_destroy(&((Camera_Manager*) c)->mutex);
            error = true;
        }
    }

    if (error) {
        cout << "Camera Manager exit" << endl;
        if (!video_file)
            close(fd);
    } else {

        try {
            // Mat frame;
            while (true) {
                cap >> ((Camera_Manager *) c) -> frame;
                
                ((Camera_Manager *) c) -> queue.push(((Camera_Manager *) c) -> frame);

                if (((Camera_Manager *) c) -> frame.empty()) { // EOF reached, rewind the video file
                    cap.release();
                    cap.open(video_device);
                    cap >> ((Camera_Manager *) c) -> frame;

                    ((Camera_Manager *) c) -> queue.push(((Camera_Manager *) c) -> frame);

                }
                ((Camera_Manager*) c) -> insert_frame(c);               
            }

            if (!video_file)
                close(fd);

        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }

    }
}

bool Camera_Manager::frames_buffer_has_elements(void *c) {
    bool ret;
    // pthread_mutex_lock(&((Camera_Manager*) c)->mutex);

    if (((Camera_Manager*) c)->frames_buffer.size() > 0)
        ret = true;
    else
        ret = false;

    // pthread_mutex_unlock(&((Camera_Manager*) c)->mutex);
    return (ret);
}

Mat Camera_Manager::get_front_frame(void* c) {
    return (((Camera_Manager*) c)->frames_buffer.front());
}

Mat Camera_Manager::get_frame(void* c) {
    // pthread_mutex_lock(&((Camera_Manager*) c)->mutex);

    ((Camera_Manager*) c) -> frame = ((Camera_Manager*) c)->frames_buffer.front();
    ((Camera_Manager*) c)->frames_buffer.erase(((Camera_Manager*) c)->frames_buffer.begin());

    // pthread_mutex_unlock(&((Camera_Manager*) c)->mutex);
    return (((Camera_Manager*) c) -> frame);

}

void *Camera_Manager::process_frames(void *c) {
    try {
        Mat image, image2, original;

        while (true) {
            pthread_mutex_lock(&((Camera_Manager*) c)->mutex);
            if (((Camera_Manager*) c) -> frames_buffer_has_elements(c)) {
                ((Camera_Manager*) c) -> get_frame(c);

                ((Camera_Manager*) c) -> frame.copyTo(image);
                ((Camera_Manager*) c) -> frame.copyTo(image2);
                ((Camera_Manager*) c) -> frame.copyTo(original);

            }
            pthread_mutex_unlock(&((Camera_Manager*) c)->mutex);
        }
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

long Camera_Manager::diff(struct timespec start, struct timespec end) {
    struct timespec temp;

    if ((end.tv_nsec - start.tv_nsec) < 0) {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = (temp.tv_sec * 1000000000) + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = (temp.tv_sec * 1000000000) + end.tv_nsec - start.tv_nsec;
    }

    return (long) temp.tv_nsec;
}

int Camera_Manager::xioctl(int fd, int request, void * arg) {
    int r;

    do {
        r = ioctl(fd, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

int Camera_Manager::print_caps(int fd) {
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)) {
        perror("Querying Capabilities");
        return 1;
    }

    printf("Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d.%d\n"
            "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version >> 16) && 0xff,
            (caps.version >> 24) && 0xff,
            caps.capabilities);

    struct v4l2_cropcap cropcap = {};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        perror("Querying Cropping Capabilities");
        return 1;
    }

    printf("Camera Cropping:\n"
            "  Bounds: %dx%d+%d+%d\n"
            "  Default: %dx%d+%d+%d\n"
            "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    struct v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        strncpy(fourcc, (char *) &fmtdesc.pixelformat, 4);
        c = (fmtdesc.flags & 1) ? 'C' : ' ';
        e = (fmtdesc.flags & 2) ? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }

    return 0;
}

int Camera_Manager::set_image_format(int fd) {
    v4l2_streamparm *setfps;

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_WIDTH;
    fmt.fmt.pix.height = IMG_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
        perror("Setting Pixel Format");
        return 1;
    }

    char fourcc[5] = {};
    strncpy(fourcc, (char *) &fmt.fmt.pix.pixelformat, 4);
    printf("Selected Camera Mode:\n"
            "  Width: %d\n"
            "  Height: %d\n"
            "  PixFmt: %s\n"
            "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);
    return 0;
}

int Camera_Manager::init_mmap(int fd) {
    struct v4l2_requestbuffers req = {};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
        perror("Querying Buffer");
        return 1;
    }

    buffer = (uint8_t *) mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);

    return 0;
}

void Camera_Manager::ChangeFrameSize(int width, int height) {
    cout << "framesize: " << width << "x" << height << endl;
    //streamer -> change_frame_size((gint) width, (gint) height);
}

void Camera_Manager::ChangeBitrate(int bitrate) {
    // streamer -> change_bitrate((gint) bitrate);
}