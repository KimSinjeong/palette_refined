#ifndef BOARD_H
#define BOARD_H

#include <opencv2/core.hpp>

enum Stone 
{
    EMPTY,
    RED,
    BLACK
};

class Board
{
public:
    Board();
    inline void updateStatus(int x, int y, Stone state) { status[x][y] = state; }
private:
    // 각 위치에 돌이 놓였는지, 누구의 돌이 놓였는지 저장
    int status[12][12];
    // Paperframe 기준으로 격자점 좌표
    cv::Point2f gridpt[12][12];
};

#endif