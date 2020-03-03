#ifndef GAME_H
#define GAME_H

#include "frame.h"
#include "board.h"
#include "ai.h"

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

#include <mutex>

using namespace std;

struct Target
{
    int px, py;
    int x, y, z = 50;
}

class GameManager
{
public:
    GameManager(ros::NodeHandle&, Frame*) {};
    void userInput();
    void Run();
    void Halt();

    // 돌을 검출하고 있는가를 나타내는 bool 변수
    bool isdetecting;

private:
    ros::NodeHandle& nodeHandler;
    ros::Publisher pub;
    ros::Rate loop_rate(20);

    bool isrunning;

    bool isfoul;

    Stone turn;
    int countturn;
    
    mutex muserinput;
    bool isupdated;
    
    Board* pboard;
    Frame* pframe;

    void putStone(Target*);

    const Target origin = {-1, -1, 0, 300, 50};
};

#endif