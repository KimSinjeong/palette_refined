#ifndef MONITOR_H
#define MONITOR_H

#include "frame.h"
#include <mutex>

class Monitor
{
public:
    Monitor(Frame* pframe_) : pframe(pframe_) {}
    void Run();
    void Halt();
    
private:
    Frame* pframe;
    std::mutex mrunthread;
    bool isrunning;

    bool isMarker();
    bool isCorner();
};

#endif