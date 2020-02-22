#ifndef MONITOR_H
#define MONITOR_H

#include "frame.h"

class Monitor
{
public:
    Monitor(Frame* mframe_) : mframe(mframe_) {}
    void Run();
    
private:
    Frame* mframe;
    

};

#endif