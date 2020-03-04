#ifndef BOARD_H
#define BOARD_H

#include "utils.h"

#include <opencv2/core.hpp>
#include <iostream>
#include <string.h>

struct Uline {
    float rho;
    float theta;
    int index = 1;
};

class Board
{
public:
    Board(int size_, int marginX_, int marginY_, int marginE_);
    //inline void updateStatus(int x, int y, Stone state) { gridpt[x][y].status = state; }
    void insert(float, float);

    // Find lines and points on the frame
    bool houghDetection();
    void findIntersections();
    bool dotDetection();
    //void confirmUserAction();
    void updateAIAction(const int, const int, int&, int&);

    void clearBoard();

    inline void setFrame(cv::Mat frame) {
        baseframe = frame(cv::Rect(marginS.x, marginS.y,
                frame.cols - marginE.x, frame.rows - marginE.y));
    }

    inline int getSize() { return size; }

    ~Board();

    // 각 위치에 돌이 놓였는지, 누구의 돌이 놓였는지 저장
    Stone **status;

private:
    // Information of each points at the go board grid
    // Upoint **gridpt;
    // baseframe 기준으로 격자점 좌표
    // (paperframe에서 marginS 만큼 원점이 이동한 좌표계)
    cv::Point2f **coord;

    // Size of Go board
    int size;

    // Frame without unecessary parts at the corners (see marginS and marginE)
    cv::Mat baseframe;

    // Margin of frame to remove a marker and pieces of points at the corners
    cv::Point2i marginS, marginE;

    // 최근에 둔 돌의 좌표 저장
    cv::Point2i recentuser;

    // Lines that are parallel to x axis, y axis, respectively.
    // (Array of lines that are composing grid)
    std::vector<Uline*> x_lines;
	std::vector<Uline*> y_lines;

    // Helper functions to find lines and points on the frame
    void findOrthogonalLines(std::vector<Uline*>&, const double, const double);
    void XYcontour(const double, const double);
    int findDotIndex(cv::KeyPoint);
    bool updateUserAction(const int, const int);
    
};

#endif