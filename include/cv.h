#ifndef H_H_CV_H_H
#define H_H_CV_H_H
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <limits>
#include <random>
#include <array>

#include "transmissionProtocol.hpp"


struct Vision {
    Vision(){
        // are seeds already setup?
        if (Vision::SEEDS[0] == Vision::SEEDS[1])
            Vision::_SETUP_SEEDS();
    }

    std::vector<Transmission> getAction(int16_t pos_x, int16_t pos_y, int16_t rotation,
                                        cv::Mat& left, cv::Mat& center, cv::Mat& right);

private:
    struct SeedVector{
        std::vector<cv::Point2i> vec;
        cv::Mat floodedImage;
        int SIZE;
    };
    std::mt19937 m_Mersenne = std::mt19937(42);

/// general data and functions
    static constexpr int IMAGES_WIDTH = 256;
    static constexpr int IMAGES_HEIGHT = 256;

    static constexpr float CAMERA_ROTATION_SCALE = 100;
    static constexpr int16_t CAMERA_ROTATION = 10 * CAMERA_ROTATION_SCALE;
    static constexpr int16_t MOVEMENTSPEED = 25; // values in percent

    static constexpr float SEED_START_Y = 0.60f;
    static constexpr int NUM_SEEDS_Y = 8;
    static constexpr int NUM_SEEDS_X = NUM_SEEDS_Y / SEED_START_Y;
    static constexpr int SEED_COUNT = NUM_SEEDS_Y * NUM_SEEDS_X;

    static constexpr int FOUND_AREA_IS_LIKLY_PATH_THRESHOLD = 4000;

    static void _SETUP_SEEDS();
    static std::array<cv::Point2i, SEED_COUNT> SEEDS;
    cv::Mat dis, left_dis, right_dis;
    SeedVector _getVectorContainingMostSeeds(const cv::Mat& center);

    struct Path{
        int16_t pos_x;
        int16_t pos_y;
        int16_t rotation;
    };
    struct AlternativePath : public Path{
        enum class NEXT_TASK { SEARCH, DODGE_LEFT, DODGE_RIGHT };
        NEXT_TASK m_NextTask;
        AlternativePath(const Path& p, NEXT_TASK t):AlternativePath::Path(p),m_NextTask(t){}
    };
    std::vector<AlternativePath> m_AlternitavePaths;
    Path m_CurrentPosition;

    bool m_OnPath = false;
    int m_IgnoreVisitedPositionCount = 100;
    std::vector<Path> m_VisitedPoints;

    Path m_OldVistedPoint = {-10000, -10000, -10000};
    void _saveCurrentPoint();
    std::vector<Transmission> _getAlternativePath();


/// follow path
    static constexpr int PATH_LOST_STEP_DISTANCE = 1000;
    static constexpr int PATH_LOST_SEED_THRESHOLD = 1500;
    static constexpr int FOLLOW_PATH_SCANLINE_HEIGHT = 0.90f * IMAGES_HEIGHT;

    int m_FollowPath_PathLostStepCounter = 0;
    int m_MultiPathCounter = 0;
    int m_SaveMultiPath_Threshold = 2;
    enum MULTIPATH_DIR { NONE = 0, LEFT = 1, RIGHT = 2, LEFT_RIGHT = LEFT | RIGHT};
    MULTIPATH_DIR m_MultiPathDirection = MULTIPATH_DIR::NONE;

    Transmission::Action m_FollowPath_LastRotation = Transmission::Action::NO_ACTION;

    std::vector<cv::Point> m_VisitedTurnings;
    std::vector<Transmission> _followPath(const SeedVector& seedGroup);

/// Search path
    enum class SEARCH_PATH_TYPE{ NO_SEARCH, MOVING, SEARCH_0_DEGREE, SEARCH_180_DEGREE,
                                 SEARCH_270_DEGREE_LEFT, SEARCH_270_DEGREE_RIGHT,
                                 SEARCH_360_DEGREE };
    SEARCH_PATH_TYPE m_SearchType = SEARCH_PATH_TYPE::SEARCH_180_DEGREE;
    float m_Search_CurrentRotation = std::numeric_limits<float>::quiet_NaN();
    bool m_Search_EnableAlternativePaths = false;
    int m_Search_RotationStart = -1.f;
    int m_Search_RotationEnd = -1.f;
    int m_Search_OldSeedSize = -1;

    std::pair<int, float> m_Search_BestSearchResult = {-1, -1.f};
    std::vector<Transmission> _searchPath(const SeedVector& seedGroup);
    void _resetSearchState();

/// Dodge objects
    static constexpr float DISPARTIY_SEARCH_X_THICKNESS = 1.f / 2.f;
    static constexpr float DISPARITY_SEARCH_HEIGHT = 0.56f;
    static constexpr uchar MIN_DODGE_BRIGHTNESS = 220;
    static constexpr int DOGE_STEP_DISTANCE = 1000;
    static constexpr int DISPARITY_OFFSET_X = 30; /// TODO set correct offset_x

    int m_Dodge_StepDistance = 0;
    Transmission::Action m_Dodge_LastDodge = Transmission::Action::NO_ACTION;
    std::vector<Transmission> _dodgeObject(const cv::Mat& disparity);
    cv::Mat m_lastDisparityMap;
};

#endif
