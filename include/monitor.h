#ifndef MONITOR_H
#define MONITOR_H

#include "frame.h"
#include "board.h"
#include "game.h"

#include <mutex>
#include <vector>

class Monitor
{
public:
    Monitor(Frame* pframe_, Board* pboard_, GameManager* pgamemanager_)
        : pframe(pframe_), pboard(pboard_), pgamemanager(pgamemanager_) {}
    void Run();
    void Halt();
    
private:
    Frame* pframe;
    Board* pboard;
    GameManager* pgamemanager;

    bool isrunning;

    bool isMarker(cv::Mat, cv::Mat&);
    bool isCorner(cv::Mat, const cv::Mat, std::vector<cv::Point2f>&);
};

#endif
