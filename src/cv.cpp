#include "cv.h"
#include <boost/range/adaptor/reversed.hpp>
#include <boost/math/constants/constants.hpp>

#define DEBUG
//#define ENABLE_IMAGE_SAVE

#ifdef DEBUG
#define IMSHOW_D(X) \
    cv::imshow(#X, X); \
    cv::waitKey(10);
#else
#define IMSHOW_D(X)
#endif

#ifdef DEBUG
#define debug(FOO) \
    FOO;
#else
#define debug(FOO);
#endif

std::array<cv::Point2i, Vision::SEED_COUNT> Vision::SEEDS = {};

std::vector<Transmission> Vision::getAction(cv::Mat &left, cv::Mat& center, cv::Mat &right) {
    cv::Mat center_gray, disparity;
    cv::cvtColor(center, center_gray, CV_RGB2GRAY);
    IMSHOW_D(center_gray);

    cv::Mat right_gray, left_gray;
    cv::cvtColor(right, right_gray, CV_BGR2GRAY);
    cv::cvtColor(left, left_gray, CV_BGR2GRAY);
    auto left_matcher = cv::StereoBM::create();
    auto right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
    auto wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    left_matcher->compute(left_gray, right_gray, left_dis);
    right_matcher->compute(right_gray, left_gray, right_dis);
    wls_filter->filter(left_dis, center, dis, right_dis);
    cv::normalize(dis, disparity, 0, 255, CV_MINMAX, CV_8UC1);

    cv::imshow("Disparity", disparity);
    cv::imshow("Center", center);
    cv::waitKey(1);
    std::vector<Transmission> actions;

    // object in front of camera?
    // only dodge if aibo is not searching for a path
    if (this->m_SearchType == Vision::SEARCH_PATH_TYPE::NO_SEARCH ||
        this->m_SearchType == Vision::SEARCH_PATH_TYPE::MOVING)
    {
        actions = this->_dogeObject(disparity);
        if (actions.size())
            return actions;
    }

    // go back on track
    if (!this->m_OnPath){
        actions = this->_searchPath(center_gray);
        if (actions.size())
            return actions;
    }

    // follow current path
    actions = this->_followPath(center_gray);
    if (actions.size())
        return actions;

    // no movement set
    // try moving forward and receive new images
    //return {{ Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED) }};
    return {{ Transmission::Action::NO_ACTION, int16_t(0) }};
}

void Vision::_SETUP_SEEDS(){
    constexpr int start_x = 0;
    constexpr int start_y = Vision::CENTER_HEIGHT * Vision::SEED_START_Y;

    constexpr float skip_y = float(Vision::CENTER_HEIGHT - start_y) / float(Vision::NUM_SEEDS_Y);
    constexpr float skip_x = float(Vision::CENTER_WIDTH) / float(Vision::NUM_SEEDS_X);

    constexpr float start_y_offset = skip_y / 2.f;
    constexpr float start_x_offset = skip_y / 2.f;

    int seed_iter = 0;
    for (float y = start_y + start_y_offset; y < Vision::CENTER_HEIGHT; y += skip_y){
    for (float x = start_x + start_x_offset; x < Vision::CENTER_WIDTH ; x += skip_x){
        Vision::SEEDS[seed_iter++] = {int(x), int(y)};
    }
    }
}

std::vector<Transmission> Vision::_followPath(const cv::Mat& center){
    std::vector<Transmission> ret;

    // path following uses a seeded approach
    // the area with the most seeds will be set as path
    std::vector<cv::Point2i> seedGroup = this->_getVectorContainingMostSeeds(center);
    if (seedGroup.size() < Vision::PATH_LOST_SEED_THRESHOLD){
        ++this->m_FollowPath_PathLostCounter;
        if (this->m_FollowPath_PathLostCounter == Vision::PATH_LOST_COUNT){
            this->m_OnPath = false;
            this->m_SearchType = Vision::SEARCH_PATH_TYPE::SEARCH_360_DEGREE;
        }
    }
    else
        this->m_FollowPath_PathLostCounter = 0;

#if defined(DEBUG) && defined(ENABLE_IMAGE_SAVE)
    if (getchar() == 'p') {
      static int counter = 0;
      std::string path("/home/bloodyangel/computerVision/");
      path += std::to_string(counter++) + ".png";
      cv::imwrite(path.c_str(), center_cpy);
    }
#endif

    // extract path directions
    std::vector<cv::Point2i> lowerLine;
    {
        int current_y = seedGroup[seedGroup.size() - 1].y;
        for (const auto& e : boost::adaptors::reverse(seedGroup)){
            if (e.y == current_y)
                lowerLine.push_back(e);
            else
                break;
        }
    }

    // simpified approach, just take mean of upper line
    float relativeMovement_x = (lowerLine[0].x + lowerLine[lowerLine.size() - 1].x) / 2 - (Vision::CENTER_WIDTH / 2);
    if (std::abs(relativeMovement_x) > 20.f){
        if (relativeMovement_x < 0.f && this->m_FollowPath_LastRotation != Transmission::Action::ROTATE_RIGHT)
            ret.emplace_back(Transmission::Action::ROTATE_LEFT, int16_t(Vision::CAMERA_ROTATION / 2));
        else if (this->m_FollowPath_LastRotation != Transmission::Action::ROTATE_LEFT)
            ret.emplace_back(Transmission::Action::ROTATE_RIGHT, int16_t(Vision::CAMERA_ROTATION / 2));
        else{}
    }
    if (std::abs(relativeMovement_x) < 40.f || !ret.size())
        ret.emplace_back(Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED));
    this->m_FollowPath_LastRotation = ret[0].action;

    // search for alternative paths
    /// mabe TODO :D

    // simplified,
    // search for alternernative paths
    return ret;
}

std::vector<cv::Point2i> Vision::_getVectorContainingMostSeeds(const cv::Mat &center) {
    cv::Mat center_cpy = center.clone();

    // cap floodfill by using a straight line
    constexpr int seed_start_height = Vision::SEED_START_Y * Vision::CENTER_HEIGHT;
    cv::line(center_cpy, {0, seed_start_height}, {center_cpy.cols, seed_start_height}, {0}, 1);

    std::array<bool, Vision::SEED_COUNT> usedSeeds;
    for (auto& e : usedSeeds)
        e = false;

    // search for an unused path color - default should be white
    uchar col = 255;
    for (bool colorIsInUse = true; colorIsInUse; --col){
        colorIsInUse = false;
        for (const auto& e : Vision::SEEDS){
            if (center_cpy.at<uchar>(e) == col){
                colorIsInUse = true;
                break;
            }
        }
    };

    // find groups
    std::vector<std::vector<cv::Point2i>> seedGroups;
    for (size_t i = 0; i < Vision::SEEDS.size(); ++i){
        if (usedSeeds[i])
            continue;

        const cv::Point2i& pos = Vision::SEEDS[i];
        cv::floodFill(center_cpy, pos, {double(col)}, nullptr, 0, 0);

        std::vector<cv::Point2i> seedGroup;
        for (size_t k = 0; k < Vision::SEEDS.size(); ++k){
            const cv::Point2i& pos = Vision::SEEDS[k];
            if (center_cpy.at<uchar>(pos) == col){
                seedGroup.push_back(pos);
                usedSeeds[k] = true;
            }
        }
        seedGroups.emplace_back(std::move(seedGroup));
        center_cpy.at<uchar>(pos) = center.at<uchar>(pos);
    }

    // find goup with the most seeds
    int highest = -1;
    std::vector<cv::Point2i>* highest_group = nullptr;
    for (auto& e : seedGroups){
        int size = e.size();
        if (highest < size){
            highest = size;
            highest_group = &e;
        }
    }

#ifdef DEBUG
    center_cpy = center.clone();
    cv::line(center_cpy, {0, seed_start_height}, {center_cpy.cols, seed_start_height}, {0}, 1);
    cv::floodFill(center_cpy, highest_group->operator[](0), {255.0}, nullptr, 0, 0);
    for (auto& e : Vision::SEEDS)
        cv::circle(center_cpy, e, 2, {0}, 1);
    cv::imshow("Seeded path", center_cpy);
    cv::waitKey(1);
#endif
    return *highest_group;
}

std::vector<Transmission> Vision::_searchPath(const cv::Mat& center){
    std::vector<Transmission> ret;
    bool searchFinished = false;

    // nan indicates the begin of a search
    if (std::isnan(this->m_Search_CurrentRotation)){
        switch (this->m_SearchType){
        case Vision::SEARCH_PATH_TYPE::SEARCH_0_DEGREE:
            this->m_Search_RotationStart = 0.f;
            this->m_Search_RotationEnd = 0.f;
            break;
        case Vision::SEARCH_PATH_TYPE::SEARCH_180_DEGREE:
            this->m_Search_RotationStart = -90.f;
            this->m_Search_RotationEnd = 90.f;
            break;
        case Vision::SEARCH_PATH_TYPE::SEARCH_360_DEGREE:
            this->m_Search_RotationStart = 0.f;
            this->m_Search_RotationEnd = 360.f;
            break;
        default:
            throw std::runtime_error("Error: search type not specified");
            break;
        }
        ret.emplace_back(Transmission::Action::ROTATE_RIGHT, this->m_Search_RotationStart);
        this->m_Search_CurrentRotation = this->m_Search_RotationStart;
        this->m_Search_BestSearchResult.first = 0;
        this->m_Doge_StepCounter = 0;
        return ret;
    }
    auto seedGroup = this->_getVectorContainingMostSeeds(center);

    if (this->m_SearchType == Vision::SEARCH_PATH_TYPE::MOVING){
        // check if VrAibo is on line
        for (const auto& e : boost::adaptors::reverse(seedGroup)){
            // if the bottom line is filled set VrAibo on line
            // last seed is on bottom line -> y value of all bottom seeds are equal
            if (e.y == Vision::SEEDS[SEED_COUNT - 1].y){
                searchFinished = true;
                break;
            }
        }

        if (!searchFinished){
            // maybe adjust direction
            // maybe TODO :D,
            // but first check results without

            // go on moving
            ret.emplace_back(Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED));
        }
    }
    else{
        if (this->m_Search_BestSearchResult.first < seedGroup.size()){
            this->m_Search_BestSearchResult.first = seedGroup.size();
            this->m_Search_BestSearchResult.second = this->m_Search_CurrentRotation;
        }

        bool startMoving = false;
        if (this->m_Search_CurrentRotation >= this->m_Search_RotationEnd)
            startMoving = true;
        else {
            this->m_Search_CurrentRotation += Vision::CAMERA_ROTATION;
            ret.emplace_back(Transmission::Action::ROTATE_RIGHT, int16_t(Vision::CAMERA_ROTATION));
        }

        if (startMoving) {
            // rotate
            ret.emplace_back(Transmission::Action::ROTATE_LEFT, int16_t(this->m_Search_CurrentRotation - this->m_Search_BestSearchResult.second));
            this->m_SearchType = Vision::SEARCH_PATH_TYPE::MOVING;
        }
    }
    if (searchFinished){
        this->m_OnPath = true;
        this->m_Search_RotationEnd = -1.f;
        this->m_Search_RotationStart = -1.f;
        this->m_SearchType = Vision::SEARCH_PATH_TYPE::NO_SEARCH;
        this->m_FollowPath_CurrentAngle += this->m_Search_BestSearchResult.second;
        this->m_Search_CurrentRotation = std::numeric_limits<float>::quiet_NaN();

        ret.emplace_back(Transmission::Action::NO_ACTION, int16_t(0));
    }
    return ret;
}

std::vector<Transmission> Vision::_dogeObject(const cv::Mat& disparity){
    std::vector<Transmission> ret;
    const int start_x_value = disparity.cols * (1.f - Vision::DISPARTIY_SEARCH_X_THICKNESS) / 2.f;
    const int max_x_value = disparity.cols - start_x_value;

    for (int y = 0; !ret.size() && y < disparity.rows * Vision::DISPARITY_SEARCH_HEIGHT; ++y){
    for (int x = start_x_value; x < max_x_value; ++x) {
        if (disparity.at<uchar>(y, x) >= Vision::MIN_DOGE_BRIGHTNESS){
            // doge left or right?
            if (this->m_Doge_LastDoge == Transmission::Action::NO_ACTION){
                if (x < disparity.cols / 2)
                    ret.emplace_back(Transmission::Action::MOVE_LEFT, 1);
                else
                    ret.emplace_back(Transmission::Action::MOVE_RIGHT, 1);
            }
            else
                ret.emplace_back(this->m_Doge_LastDoge, 1);
            break;
        }
    }
    }
    //this->m_LastDoge = Transmission::Action::NO_ACTION;

    if (ret.size()) {
        this->m_Doge_LastDoge = ret[0].action;
        this->m_Doge_StepCounter = 0;
        this->m_OnPath = false;
        if (this->m_SearchType == Vision::SEARCH_PATH_TYPE::MOVING)
            this->m_SearchType =  Vision::SEARCH_PATH_TYPE::SEARCH_180_DEGREE;
    }
    else if (this->m_Doge_LastDoge != Transmission::Action::NO_ACTION){
        if (this->m_Doge_StepCounter < Vision::STEP_COUNTER){
            this->m_Doge_StepCounter++;
            ret.emplace_back(Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED));
        }
        else{
            this->m_SearchType = Vision::SEARCH_PATH_TYPE::SEARCH_180_DEGREE;
            this->m_Doge_LastDoge = Transmission::Action::NO_ACTION;
            this->m_OnPath = false;
        }
    }
    else{}
    return ret;
}
#undef IMSHOW_D
#undef debug
