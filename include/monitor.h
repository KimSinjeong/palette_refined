#ifndef MONITOR_H
#define MONITOR_H

#include "frame.h"
#include "board.h"
#include <mutex>

class Monitor
{
public:
    Monitor(Frame* pframe_, Board* pboard_)
        : pframe(pframe_), pboard(pboard_) {}
    void Run();
    void Halt();
    
private:
    Frame* pframe;
    Board* pboard;

    bool isrunning;

    bool isMarker();
    bool isCorner();
};

#endif