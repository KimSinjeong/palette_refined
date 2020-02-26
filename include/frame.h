#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core.hpp>
#include <vector>
#include <string>

class SubFrame
{
public:
    SubFrame() { }
    SubFrame(cv::Mat frame_);

    // Getters
    inline cv::Mat& getFrame() { return frame; }
    inline cv::Point2f* getReferential() { return referentialpt; }
    inline cv::Point2f* getMarker() { return markerpt; }

    // Setters
    inline void setFrame(cv::Mat& frame_) { frame = frame_; }
    void setReferential(std::vector<cv::Point2d> pt_);
    void setReferential(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);
    void setMarker(std::vector<cv::Point2d> pt_);
    void setMarker(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);
    inline void setMarker(float x, float y, int idx) { markerpt[idx] = cv::Point2f(x, y); }

private:
    // Frame matrix contains all the pixels.
    cv::Mat frame;

    // Each point array contains 4 referential points of the subframe.
    // Thus corresponding points between frames will be used to get perspective transformation matrix.
    // Index increases when goes clockwise.
    /*
        TODO: 만약 getPerspectiveTransform이 vector<Point2d>도 입력으로 받는다면
            아예 아래의 정의를 vector로 바꿔 버리기.
    */
    cv::Point2f referentialpt[4];
    cv::Point2f markerpt[4];
};

class Frame
{
public:
    Frame() { }
    Frame(cv::Mat image);

    // global marker의 네 꼭짓점의 Global coordinate 기준 좌표를 입력하는 함수.
    // [x1, y1, x2, ...] 순서로 넘겨줘야 함.
    bool setGlobalMarkerPos(float pt[]);
    // Paper frame의 크기를 얼마로 잡을지 설정하는 함수; 그대로 imageframe의 localpt가 된다.
    void setPaperFrameSize(int width, int height);

    inline void setImage(cv::Mat image) { imageframe.setFrame(image); }

    inline void img2paperRelation() {
        imagetopapertf = cv::getPerspectiveTransform(imageframe.getReferential(), paperframe.getReferential());
    }

    inline void global2paperRelation() {
        papertoglobaltf = cv::getPerspectiveTransform(paperframe.getMarker(), globalframe.getMarker());
    }

    void calculateRelations(std::vector<cv::Point2f> pBlob);

    // Get the matrix of indicated target Subframe.
    Mat& getFrame(std::string target);

    // Set size of paperframe
    inline void setPaperSize(int s) {
        papersize = s;
        paperframe.setReferential(cv::Point2f(0, 0), cv::Point2f((float)s, 0),
            cv::Point2f((float)s, (float)s), cv::Point2f(0, (float)s));
    }

    inline int getPaperSize() { return papersize; }

    // Perspective transformation matrices
    cv::Mat imagetopapertf;
    cv::Mat papertoglobaltf;

private:
    // Image frame에는 전체 이미지
    // referentialpt - ID:0의 1번 꼭짓점, 바둑판의 다른 세 꼭짓점의 좌표
    // markerpt - ID:0의 네 꼭짓점 좌표
    SubFrame imageframe;
    // Paper frame에는 바둑판 영역 이미지 (원점)
    // referentialpt - ID:0의 1번 꼭짓점, 바둑판의 다른 세 꼭짓점의 좌표
    // markerpt - ID:41의 네 꼭짓점 좌표
    SubFrame paperframe;
    // Global frame에는 저장 안함
    // markerpt - ID:41의 네 꼭짓점 좌표
    SubFrame globalframe;

    int papersize;

    cv::Point2f image2papertr;
};

#endif