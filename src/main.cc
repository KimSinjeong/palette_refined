/*
    KAIST robot circle MR, 2019 ~ 2020 4-dof robot arm project
    <Palette>
    main.cc
    Run threads for main works and communicate with robot.
    There are 4 main works :
        1. Get current camera frame and cut square board from it.
        2. Determine where the user put a stone. (in paper frame)
        3. Determine where the robot should put a stone. (in global frame)
        4. Move the robot to target points via ROS message.
    There are 3 types of frames : 
        1. Camera frame (in image frame)
        2. Paper frame based on QR code with ID:0
        3. Global frame based on QR code with ID:41
*/

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "monitor.h"
#include "frame.h"
#include "board.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(Frame* frame) : pframe(frame) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

private:
    Frame* pframe;
    mutex mreadimage;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mainframe_node");
    ros::start();

    // Read arguments
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "Wrong path to settings" << std::endl;
        return 1;
    }

    int papersize, marginx, marginy, margine, boardsize;
    vector<cv::Point2f> globalmarker(4);
    fsSettings["Paper.size"] >> papersize; fsSettings["Board.size"] >> boardsize;
    fsSettings["Paper.marginX"] >> marginx; fsSettings["Paper.marginY"] >> marginy;
    fsSettings["Paper.marginE"] >> margine;
    
    fsSettings["Board.p0"] >> globalmarker[0];
    fsSettings["Board.p1"] >> globalmarker[1];
    fsSettings["Board.p2"] >> globalmarker[2];
    fsSettings["Board.p3"] >> globalmarker[3];

    // Initialize objects
    Frame mainframe;
    mainframe.setPaperSize(papersize);
    mainframe.globalframe.setReferential(globalmarker);

    Board goboard(boardsize, marginx, marginy, margine);

    // TODO: Create classes
    Monitor monitor(&mainframe, &goboard);
    // Intelligence intelligence = Intelligence(...);
    // RobotMover robotmover = RobotMover(...);

    // TODO: Run threads
    thread tMonitor = thread(&Monitor::Run, &monitor);
    // thread tIntelligence = thread(...);
    // thread tRobotMover = thread(...);

    ImageGrabber igb(&mainframe);

    ros::NodeHandle nodeHandler;
    image_transport::ImageTransport it(nodeHandler);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // When linking error about OpenCV, refer to link below.
    // https://answers.ros.org/question/257581/how-to-use-arbitrary-version-of-opencv/

    // TODO: Halt threads
    monitor.Halt();
    // intelligence.halt();
    // robotmover.halt();

    tMonitor.join();

    ros::shutdown();

    return 0;
}

// Function which get a current camera frame.
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Set mutex when reading an image.
    {
        unique_lock<mutex> lock(mreadimage);
        pframe->setImage(cv_ptr->image);

        // For Debug
        cv::imshow("testImage", cv_ptr->image);
        cv::waitKey(1);
    }
}
