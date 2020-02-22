#ifndef BOARD_H
#define BOARD_H

#include <opencv2/core.hpp>
#include <iostream>

enum Stone 
{
    EMPTY,
    RED,
    BLACK
};

class Board
{
public:
    Board(int size) : size(size) {}
    inline void updateStatus(int x, int y, Stone state) { status[x][y] = state; }
    // Determine whether there is a winner; if so, return the color of winner.
    bool isWinner(Stone player);
    void clearBoard();

private:
    // 각 위치에 돌이 놓였는지, 누구의 돌이 놓였는지 저장
    Stone status[12][12];
    // Paperframe 기준으로 격자점 좌표
    cv::Point2f gridpt[12][12];
    // 바둑판 크기
    int size;
};

#endif