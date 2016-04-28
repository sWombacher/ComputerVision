#include "cv.h"


Vision::Action Vision::getAction(cv::Mat &left, cv::Mat &center, cv::Mat &right) const
{
    cv::Mat disparity;
    cv::Mat left_gray;
    cv::Mat right_gray;
    cv::cvtColor(right, right_gray, CV_BGR2GRAY);
    cv::cvtColor(left, left_gray, CV_BGR2GRAY);
    auto sbm = cv::StereoBM::create();
    sbm->compute(left_gray, right_gray, disparity);

    cv::Mat tmp1;
    cv::normalize(disparity, tmp1, 0, 255, CV_MINMAX, CV_8UC1);
    cv::imshow("Disparity", tmp1);
    return Vision::Action::MOVE_FORWARD;
}
