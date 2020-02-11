#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core.hpp>
#include <vector>

class Frame
{
public:
    Frame();

private:
    int badukpan[12][12];
    // Image frame에는 전체 이미지, ID:0의 1번 귀퉁이, 바둑판의 다른 세 귀퉁이점의 좌표 저장.
    subFrame imageframe;
    // 
    subFrame paperframe;
    subFrame globalframe;
};


class subFrame
{
public:
    subFrame();
    subFrame(const subFrame& origin);

    // Getters
    cv::Mat getFrame();
    cv::Point2f* getpts();

    // Setters
    void setFrame(cv::Mat frame_);
    void setpts(std::vector<cv::Point2d> pt_);
    void setpts(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);

private:
    // Frame matrix contains all the pixels.
    cv::Mat frame;

    // Point array contains 4 referential points of the subframe.
    // Thus corresponding points between frames will be used to get perspective transformation matrix.
    /*
        TODO: 만약 getPerspectiveTransform이 vector<Point2d>도 입력으로 받는다면
            아예 아래의 정의를 vector로 바꿔 버리기.
    */
    cv::Point2f pt[4];
};

#endif