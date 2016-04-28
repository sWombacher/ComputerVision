#ifndef H_H_CV_H_H
#define H_H_CV_H_H
#pragma once
#include <opencv2/opencv.hpp>

struct Vision {
    enum class Action {
        NO_ACTION,
        CLOSE_CONNECTION, DISCONNECTED,
        INC_SPEED, DEC_SPEED,
        MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
        ROTATE_LEFT, ROTATE_RIGHT,
        HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN
    };
    Action getAction(cv::Mat& left, cv::Mat& center, cv::Mat& right) const;
};

#endif
