/*
    KAIST robot circle MR, 2019 ~ 2020 4-dof robot arm project
    <Palette>
    main.cc
    Run threads for main works and communicate with robot.
    There are 4 main works :
        1. Get current camera frame and cut square board from it.
        2. Determine where the user put a stone.
        3. Determine where the robot should put a stone. (in global frame)
        4. Move the robot to target points via ROS message.
    There are 3 types of frames : 
        1. Camera frame (pixel frame)
        2. Paper frame based on QR code with ID:0
        3. Global frame based on QR code with ID:41
*/

#include "frame.h"

int main()
{
    // TODO: Run threads
    do {
        // TODO: Get current frame via ROS messages.
    } while(true);

    return 0;
}

// TODO: Function which get a current camera frame.