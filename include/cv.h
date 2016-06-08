#ifndef H_H_CV_H_H
#define H_H_CV_H_H
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <limits>
#include <array>

#include "transmissionProtocol.hpp"


struct Vision {
    Vision(){
        // are seeds already setup?
        if (Vision::SEEDS[0] == Vision::SEEDS[1])
            Vision::_SETUP_SEEDS();
    }

    std::vector<Transmission> getAction(cv::Mat& left, cv::Mat& center, cv::Mat& right);

private:
    struct SeedVector{
        std::vector<cv::Point2i> vec;
        int PIXEL_SIZE;
    };
    struct AlternativePath{
        cv::Point2f pos;
        float angle;
    };
    bool m_OnPath = true;

    /// general data and functions
    static constexpr int CENTER_WIDTH = 256;
    static constexpr int CENTER_HEIGHT = 256;

    static constexpr float CAMERA_ROTATION = 10.f;
    static constexpr float MOVEMENTSPEED = 1.f;

    static constexpr float SEED_START_Y = 0.6f;
    static constexpr int NUM_SEEDS_Y = 8;
    static constexpr int NUM_SEEDS_X = NUM_SEEDS_Y / SEED_START_Y;
    static constexpr int SEED_COUNT = NUM_SEEDS_Y * NUM_SEEDS_X;

    static void _SETUP_SEEDS();
    static std::array<cv::Point2i, SEED_COUNT> SEEDS;
    cv::Mat dis, left_dis, right_dis;
    SeedVector _getVectorContainingMostSeeds(const cv::Mat& center);


    /// follow path
    static constexpr int PATH_LOST_COUNT = 3;
    static constexpr int PATH_LOST_SEED_THRESHOLD = 10;

    int m_FollowPath_PathLostCounter = 0;
    float m_FollowPath_CurrentAngle = 0.f;
    Transmission::Action m_FollowPath_LastRotation = Transmission::Action::NO_ACTION;

    std::vector<AlternativePath> m_Alternatives;
    std::vector<cv::Point> m_VisitedTurnings;

    std::vector<Transmission> _followPath(const cv::Mat& center);

    /// Search path
    enum class SEARCH_PATH_TYPE{ NO_SEARCH, MOVING, SEARCH_0_DEGREE, SEARCH_180_DEGREE, SEARCH_360_DEGREE };
    SEARCH_PATH_TYPE m_SearchType = SEARCH_PATH_TYPE::NO_SEARCH;
    float m_Search_CurrentRotation = std::numeric_limits<float>::quiet_NaN();
    int m_Search_RotationStart = -1.f;
    int m_Search_RotationEnd = -1.f;

    std::pair<size_t, float> m_Search_BestSearchResult = {0, -1.f};
    std::vector<Transmission> _searchPath(const cv::Mat& center);

    /// Doge objects
    static constexpr float DISPARTIY_SEARCH_X_THICKNESS = 10.f / 100.f;
    static constexpr float DISPARITY_SEARCH_HEIGHT = 0.5f;
    static constexpr uchar MIN_DOGE_BRIGHTNESS = 220;
    static constexpr int STEP_COUNTER = 6;
    static constexpr int DISPARITY_OFFSET_X = 30; /// TODO set correct offset_x

    int m_Doge_StepCounter = 0;
    Transmission::Action m_Doge_LastDoge = Transmission::Action::NO_ACTION;
    std::vector<Transmission> _dogeObject(const cv::Mat& disparity);
    cv::Mat m_lastDisparityMap;
};

#endif
