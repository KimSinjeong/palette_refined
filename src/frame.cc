#include "frame.h"
#include <iostream>

#include <opencv2/imgproc.hpp>

bool Frame::setGlobalMarkerPos(float pt[])
{
    try {
        for(int i = 0; i < 4; i++)
        {
            globalframe.setMarker(pt[2*i], pt[2*i+1], i);
        }
    } catch(const std::out_of_range& e){
        std::cerr << "setGlobalMarkerPos exception: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void Frame::setPaperFrameSize(int width, int height)
{
    paperframe.setReferential(cv::Point2f(0.f, 0.f), cv::Point2f((float)width, 0.f),
        cv::Point2f((float)width, (float)height), cv::Point2f(0.f, (float)height));
}

void Frame::calculateRelations(std::vector<cv::Point2f>& pBlob)
{
    // image to paper tf 구하기
    pBlob[0] = imageframe.getMarker()[0];
    imageframe.setReferential(pBlob);
    img2paperRelation();

    warpPerspective(imageframe.getFrame(), paperframe.getFrame(),
            imagetopapertf, cv::Size(getPaperSize(), getPaperSize()));

    // This array is length 4
    cv::Point2f* gmarker = paperframe.getMarker();
    std::vector<cv::Point2f> src(gmarker, gmarker+4);

    // paper to global tf 구하기
    perspectiveTransform(src, gmarker, imagetopapertf);
    paper2globalRelation();
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