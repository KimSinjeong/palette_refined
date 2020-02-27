#ifndef BOARD_H
#define BOARD_H

#include <opencv2/core.hpp>
#include <iostream>
#include <string.h>

enum Stone 
{
    EMPTY,
    RED,
    BLACK
};

struct Uline {
    float rho;
    float theta;
    int index = 1;
}

struct Upoint {
    // 각 위치에 돌이 놓였는지, 누구의 돌이 놓였는지 저장
    Stone status;
    // baseframe 기준으로 격자점 좌표
    // (paperframe에서 marginS 만큼 원점이 이동한 좌표계)
    cv::Point2f coord;
}

class Board
{
public:
    Board(int size_, int marginX_, int marginY_, int marginE_);
    inline void updateStatus(int x, int y, Stone state) { gridpt.status[x][y] = state; }
    void insert(float, float);

    // Find lines and points on the frame
    bool houghDetection();
    void findIntersections();
    bool dotDetection();
    void confirmUserAction();

    // Determine whether there is a winner; if so, return the color of winner.
    bool isWinner(Stone player);
    void clearBoard();

    inline void setFrame(cv::Mat frame) {
        baseframe = frame(cv::Rect(marginS.x, marginS.y,
                frame.cols - marginE.x, frame.rows - marginE.y));
    }

    ~Board();

private:
    // 각 격자점의 정보
    Upoint **gridpt;
    // 바둑판 격자 선
    Uline **gridline;
    // 바둑판 크기
    int size;

    cv::Mat baseframe;

    cv::Point2i marginS, marginE;

    // 최근에 둔 돌이 완전히 처리되고 나면 -1, -1 이 된다.
    cv::Point2i recentuser;

    // Lines that are parallel to x axis, y axis, respectively.
    std::vector<Uline*> x_lines;
	std::vector<Uline*> y_lines;

    // Helper functions to find lines and points on the frame
    void findOrthogonalLines(std::vector<Uline*>&, const double, const double);
    void XYcontour(const double, const double);
    int findDotIndex(cv::KeyPoint);
    bool updateUserAction(const int, const int);
    
};

#endif