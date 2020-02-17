#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv){
    // Check if video source has been passed as a parameter
    if(argc != 3)
    {
        std::cerr << std::endl << "Usage ros_palette camera_node CAMERA_DEVICE_NO PATH_TO_SETTINGS";
        return 1;
    }
  
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
    
    int cam_device;

    // Convert the passed as command line parameter index
    //for the video device to an integer
    std::istringstream cam_deviceCmd(argv[1]);
    // Check if it is indeed a number
    if(!(cam_deviceCmd >> cam_device)) {
        std::cerr << std::endl << "Camera device number should be an integer";    
        return 1;
    }

    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        std::cerr << std::endl << "Wrong paht to settings";
        return 1;
    }

    double fx, fy, cx, cy; // Camera matrix
    double k1, k2, p1, p2; // Distortion

    fsSettings["Camera.fx"] >> fx;
    fsSettings["Camera.fy"] >> fy;
    fsSettings["Camera.cx"] >> cx;
    fsSettings["Camera.cy"] >> cy;
    fsSettings["Camera.k1"] >> k1;
    fsSettings["Camera.k2"] >> k2;
    fsSettings["Camera.p1"] >> p1;
    fsSettings["Camera.p2"] >> p2;

    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);

    //initialize a VideoCapture variable to get camera data
    cv::VideoCapture cap(cam_device);
    // Check if video device can be opened with the given index
    if(!cap.isOpened()) return 1;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
  
    while (nh.ok()) {
        cap >> frame;
        //Check if grabbed frame is actually full with some content,
        //then publish the image
        if(!frame.empty()) {
            // TODO: Apply camera matrix and undistort frame
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
    }
} 
