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
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "frame.h"

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

    // TODO: Create classes and Run threads
    // BoardMonitor boardmonitor = BoardMonitor(...);
    // Intelligence intelligence = Intelligence(...);
    // RobotMover robotmover = RobotMover(...);

    // TODO: Run threads
    // thread ptBoardMonitor = thread(...);
    // thread ptIntelligence = thread(...);
    // thread ptRobotMover = thread(...);

    Frame mainframe = Frame();

    ImageGrabber igb(&mainframe);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // TODO: 2/16
    // 0. Restore palette package and CMakelists.txt file
    // 1. Try to re-install palette package
    // 2. Try to remove palette package
    // 3. Try to remove ROS
    // https://answers.ros.org/question/257581/how-to-use-arbitrary-version-of-opencv/

    // TODO: Halt threads
    // boardmonitor.halt();
    // intelligence.halt();
    // robotmover.halt();

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
