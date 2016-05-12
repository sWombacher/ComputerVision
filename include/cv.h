#ifndef H_H_CV_H_H
#define H_H_CV_H_H
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <array>


struct Vision {
    enum class Action {
        NO_ACTION,
        CLOSE_CONNECTION, DISCONNECTED,
        INC_SPEED, DEC_SPEED,
        MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
        ROTATE_LEFT, ROTATE_RIGHT,
        HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN
    };
    Vision(){
        // are seeds already setup?
        if (Vision::SEEDS[0] == Vision::SEEDS[1])
            Vision::_SETUP_SEEDS();
    }

    Action getAction(cv::Mat& left, cv::Mat& center, cv::Mat& right);

private:
    struct AlternativePath{
        cv::Point2f pos;
        float angle;
    };

    bool m_OnPath = true;

    std::vector<AlternativePath> m_Alternatives;
    std::vector<cv::Point> m_VisitedTurnings;

    static constexpr int CENTER_WIDTH = 256;
    static constexpr int CENTER_HEIGHT = 256;

    /// follow path
    static constexpr float SEED_START_Y = 0.6f;
    static constexpr int NUM_SEEDS_Y = 8;
    static constexpr int NUM_SEEDS_X = NUM_SEEDS_Y / SEED_START_Y;
    static constexpr int SEED_COUNT = NUM_SEEDS_Y * NUM_SEEDS_X;
    float m_CurrentAngle = 0.f;

    static void _SETUP_SEEDS();
    static std::array<cv::Point2i, SEED_COUNT> SEEDS;

    Action _followPath(const cv::Mat& center);
    std::vector<cv::Point2i> _getVectorContainingMostSeeds(const cv::Mat& center);

    /// Search path
    Action _searchPath(const cv::Mat& center);

    /// Doge objects
    static constexpr float DISPARITY_SEARCH_HEIGHT = 0.6f;
    static constexpr float DISPARTIY_SEARCH_X_THICKNESS = 1.f / 100.f;
    static constexpr uchar MIN_DOGE_BRIGHTNESS = 150;
    static constexpr int STEP_COUNTER = 10;

    Action m_LastDoge = Action::NO_ACTION;
    Action _dogeObject(const cv::Mat& disparity);
    int m_StepCounter = 0;

    cv::Mat dis, left_dis, right_dis;
};

#endif
