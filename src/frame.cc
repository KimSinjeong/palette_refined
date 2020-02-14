#include "frame.h"
#include <iostream>

Frame::Frame(cv::Mat image) : imageframe(image)
{
    // TODO: 들어온 이미지에서 마커 및 바둑판 꼭짓점 검출 등의 작업이 이루어져야 함
    // Caution: 위에서 언급한 작업이 main에서 이뤄지는 편이 타당할 수도 있음
}

bool Frame::setGlobalMarkerPos(float pt[])
{
    try {
        for(int i = 0; i < 4; i++)
        {
            globalframe.setMarker(pt[2*i], pt[2*i+1], i);
        }
    } catch(std::out_of_range& e){
        std::cerr << "ERROR : " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Frame::setPaperFrameSize(int width, int height)
{
    paperframe.setReferential(cv::Point2f(0.f, 0.f), cv::Point2f((float)width, 0.f),
        cv::Point2f((float)width, (float)height), cv::Point2f(0.f, (float)height));
}

SubFrame::SubFrame(cv::Mat frame_) : frame(frame_) {}


void SubFrame::setReferential(std::vector<cv::Point2d> pt_)
{
    for(int i = 3; i >= 0; i--)
    {
        referentialpt[i] = pt_.back();
        pt_.pop_back();
    }
}

void SubFrame::setReferential(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d)
{
    referentialpt[0] = a; referentialpt[1] = b; referentialpt[2] = c; referentialpt[3] = d;
}

void SubFrame::setMarker(std::vector<cv::Point2d> pt_)
{
    for(int i = 3; i >= 0; i--)
    {
        markerpt[i] = pt_.back();
        pt_.pop_back();
    }
}

void SubFrame::setMarker(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d)
{
    markerpt[0] = a; markerpt[1] = b; markerpt[2] = c; markerpt[3] = d;
}