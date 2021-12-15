#include "camera_manager.h"
#include "gstreamer.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>



int main(int argc, char** argv) {
   
    string ip_camera_process = "127.0.0.1"; // default viewer IP
    string device_name = "/dev/video0"; // default video device
    unsigned short int port_camera_process = 5200;

    setlocale(LC_ALL, "En_US");


    setNumThreads(8);

    
   
    gst_init(NULL, NULL);
    
    


    // camera_manager sends images using network thought ip_camera_process and port_camera_process 
    Camera_Manager camera_manager(device_name, ip_camera_process, port_camera_process);
    
    std::cout << "Starting Gstreamer" << std::endl;
    
    // gstreamer g(&camera_manager, ip_camera_process);
    // g.start_UDP_gstreaming(&g);
    // g.start_RTSP_gstreaming(&g);

    return 0;
}



