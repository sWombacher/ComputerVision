#include "cv.h"
#include <boost/range/adaptor/reversed.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>

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
    cv::Mat disparity;
    IMSHOW_D(center);

    auto left_matcher = cv::StereoBM::create();
    auto right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
    auto wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

    left_matcher->compute(left, right, left_dis);
    right_matcher->compute(right, left, right_dis);
    wls_filter->filter(left_dis, center, dis, right_dis);
    cv::normalize(dis, disparity, 0, 255, CV_MINMAX, CV_8UC1);

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
        actions = this->_searchPath(center);
        if (actions.size())
            return actions;
    }

    // follow current path
    actions = this->_followPath(center);
    if (actions.size())
        return actions;

    // no movement set
    // try moving forward and receive new images
    //return {{ Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED) }};
    return {{ Transmission::Action::NO_ACTION, int16_t(0) }};
}

void Vision::_SETUP_SEEDS(){
    constexpr int start_x = 0;
    constexpr int start_y = Vision::IMAGES_HEIGHT * Vision::SEED_START_Y;

    constexpr float skip_y = float(Vision::IMAGES_HEIGHT - start_y) / float(Vision::NUM_SEEDS_Y);
    constexpr float skip_x = float(Vision::IMAGES_WIDTH) / float(Vision::NUM_SEEDS_X);

    constexpr float start_y_offset = skip_y / 2.f;
    constexpr float start_x_offset = skip_y / 2.f;

    int seed_iter = 0;
    for (float y = start_y + start_y_offset; y < Vision::IMAGES_HEIGHT; y += skip_y){
    for (float x = start_x + start_x_offset; x < Vision::IMAGES_WIDTH ; x += skip_x){
        Vision::SEEDS[seed_iter++] = {int(x), int(y)};
    }
    }
}

std::vector<Transmission> Vision::_followPath(const cv::Mat& center){
    std::vector<Transmission> ret;

    // path following uses a seeded approach
    // the area with the most seeds will be set as path
    SeedVector seedGroup = this->_getVectorContainingMostSeeds(center);

    auto pathLost = [this, &ret]() mutable {
        ++this->m_FollowPath_PathLostCounter;
        if (this->m_FollowPath_PathLostCounter == Vision::PATH_LOST_COUNT){
            this->m_OnPath = false;
            this->m_FollowPath_PathLostCounter = 0;
            this->m_SearchType = Vision::SEARCH_PATH_TYPE::SEARCH_360_DEGREE;
        }
        ret.emplace_back(Transmission::Action::MOVE_FORWARD, int16_t(Vision::MOVEMENTSPEED));
    };

    if (seedGroup.PIXEL_SIZE < Vision::PATH_LOST_SEED_THRESHOLD){
        debug(std::cout << "Path lost seedgroup to small" << std::endl);
        pathLost();
        return ret;
    }

#if defined(DEBUG) && defined(ENABLE_IMAGE_SAVE)
    if (getchar() == 'p') {
      static int counter = 0;
      std::string path("/home/bloodyangel/computerVision/");
      path += std::to_string(counter++) + ".png";
      cv::imwrite(path.c_str(), center_cpy);
    }
#endif

    // create scan line
    std::vector<int> pathMidPoints;
    {
        for (int i = 0, oldPoint = std::numeric_limits<int>::min(); i < seedGroup.floodedImage.cols; ++i){
            if (seedGroup.floodedImage.at<uchar>(Vision::FOLLOW_PATH_SCANLINE_HEIGHT, i) == 0){
                // path point found
                if (oldPoint != i - 1)
                    pathMidPoints.push_back(i); // new path found
                oldPoint = i;
            }
            else{
                if (oldPoint == i - 1 && pathMidPoints.size()){
                    // adjust point to be actual in the middle
                    pathMidPoints[pathMidPoints.size() - 1] += i-1;
                    pathMidPoints[pathMidPoints.size() - 1] /= 2;
                }
            }
        }
        // check last point on path, which hasn't been checked
        if (seedGroup.floodedImage.at<uchar>(Vision::FOLLOW_PATH_SCANLINE_HEIGHT, seedGroup.floodedImage.cols - 1) == 0){
            pathMidPoints[pathMidPoints.size() - 1] += seedGroup.floodedImage.cols - 1;
            pathMidPoints[pathMidPoints.size() - 1] /= 2;
        }
    }

#ifdef DEBUG
    cv::Mat& tmp = seedGroup.floodedImage;
    cv::line(tmp, {0, Vision::FOLLOW_PATH_SCANLINE_HEIGHT}, {tmp.cols - 1, Vision::FOLLOW_PATH_SCANLINE_HEIGHT}, {127}, 1);
    cv::imshow("Scanline", tmp);
    cv::waitKey(10);
#endif

    if (pathMidPoints.size()){
        if (pathMidPoints.size() > 1){
            // multiple paths found
            std::cout << "Multi paths" << std::endl;
        }
        else{
            // only one path found
        }
    }
    else{
        // no path found
        debug(std::cout << "Path lost on scanline" << std::endl);
        pathLost();
        return ret;
    }
    float relativeMovement_x = pathMidPoints[0] - seedGroup.floodedImage.cols / 2;

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

    // path found
    this->m_FollowPath_PathLostCounter = 0;
    return ret;
}

Vision::SeedVector Vision::_getVectorContainingMostSeeds(const cv::Mat &center) {
    cv::Mat center_cpy = center.clone();

    // cap floodfill by using a straight line
    constexpr int seed_start_height = Vision::SEED_START_Y * Vision::IMAGES_HEIGHT;
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
    SeedVector bestGroup;
    for (size_t i = 0, currentHighestPIxelCount = 0; i < Vision::SEEDS.size(); ++i){
        if (usedSeeds[i])
            continue;

        const cv::Point2i& pos = Vision::SEEDS[i];
        int pixelCount = cv::floodFill(center_cpy, pos, {double(col)}, nullptr, 0, 0);

        std::vector<cv::Point2i> seedGroup;
        for (size_t k = 0; k < Vision::SEEDS.size(); ++k){
            const cv::Point2i& pos = Vision::SEEDS[k];
            if (center_cpy.at<uchar>(pos) == col){
                seedGroup.push_back(pos);
                usedSeeds[k] = true;
            }
        }
        if (currentHighestPIxelCount < size_t(pixelCount)) {
            currentHighestPIxelCount = pixelCount;
            bestGroup.vec = std::move(seedGroup);
            bestGroup.PIXEL_SIZE = pixelCount;
        }
        //center_cpy.at<uchar>(pos) = col + 1;
        cv::floodFill(center_cpy, pos, {double(col+1)}, nullptr, 0, 0);
    }

    cv::Mat& output = bestGroup.floodedImage;
    output = center.clone();
    cv::line(output, {0, seed_start_height}, {center_cpy.cols, seed_start_height}, {0}, 1);
    cv::floodFill(output, bestGroup.vec[0], {0.0}, nullptr, 0, 0);
    cv::threshold(output, output, 0, 255, cv::THRESH_BINARY);

#ifdef DEBUG
/*

    using namespace cv;
    using namespace std;
    cv::subtract(cv::Scalar::all(255), output, center_cpy);
    cv::imshow("why?", center_cpy);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;


    /// Find contours
      static RNG rng(12345);
      findContours( center_cpy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, Point(0, 0) );

      /// Approximate contours to polygons + get bounding rects and circles
      vector<vector<Point> > contours_poly( contours.size() );
      vector<Rect> boundRect( contours.size() );

      for( int i = 0; i < contours.size(); i++ )
         { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
           boundRect[i] = boundingRect( Mat(contours_poly[i]) );
           //minEnclosingCircle( (Mat)contours_poly[i], center1[i], radius[i] );
         }


      /// Draw polygonal contour + bonding rects + circles
      Mat drawing = Mat::zeros( center_cpy.size(), CV_8UC3 );
      //for( int i = 0; i< contours.size(); i++ )
      int i = 0;
         {
           Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
           drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
           rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
           //circle( drawing, center1[i], (int)radius[i], color, 2, 8, 0 );
         }
    cv::imshow("bamlee", drawing);
*/

    center_cpy = output.clone();
    cv::circle(center_cpy,bestGroup.vec[0], 3, {127});
    cv::waitKey(10);

    for (auto& e : Vision::SEEDS)
        cv::circle(center_cpy, e, 2, {0}, 1);

    cv::imshow("Flooded", output);
    cv::imshow("Seeded path", center_cpy);
    cv::waitKey(10);
#endif

    return bestGroup;
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
        this->m_Doge_StepDistance = 0;
        return ret;
    }
    auto seedGroup = this->_getVectorContainingMostSeeds(center);

    if (this->m_SearchType == Vision::SEARCH_PATH_TYPE::MOVING){
        // check if VrAibo is on line
        for (const auto& e : boost::adaptors::reverse(seedGroup.vec)){
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
        if (this->m_Search_BestSearchResult.first < seedGroup.PIXEL_SIZE){
            this->m_Search_BestSearchResult.first = seedGroup.PIXEL_SIZE;
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
    const int start_y_value = disparity.rows * Vision::DISPARITY_SEARCH_HEIGHT / 2.f;
    int start_x_value = disparity.cols * (1.f - Vision::DISPARTIY_SEARCH_X_THICKNESS) / 2.f;
    const int max_x_value = disparity.cols - start_x_value + Vision::DISPARITY_OFFSET_X;
    start_x_value += Vision::DISPARITY_OFFSET_X;

#ifdef DEBUG
    cv::Mat tmp = disparity.clone();
    cv::rectangle(tmp, {start_x_value, start_y_value},
                       {max_x_value, int(disparity.rows * Vision::DISPARITY_SEARCH_HEIGHT)},
                       {255});
    cv::imshow("Search rect", tmp);
    cv::waitKey(10);
#endif

    for (int y = 0; !ret.size() && y < disparity.rows * Vision::DISPARITY_SEARCH_HEIGHT; ++y){
    for (int x = start_x_value; x < max_x_value; ++x) {
        if (disparity.at<uchar>(y, x) >= Vision::MIN_DOGE_BRIGHTNESS){
            // doge left or right?
            if (this->m_Doge_LastDoge == Transmission::Action::NO_ACTION){
                if (x < disparity.cols / 2)
                    ret.emplace_back(Transmission::Action::MOVE_LEFT, int16_t(Vision::MOVEMENTSPEED));
                else
                    ret.emplace_back(Transmission::Action::MOVE_RIGHT, int16_t(Vision::MOVEMENTSPEED));
            }
            else
                ret.emplace_back(this->m_Doge_LastDoge, int16_t(Vision::MOVEMENTSPEED));
            break;
        }
    }
    }
    //this->m_LastDoge = Transmission::Action::NO_ACTION;

    if (ret.size()) {
        this->m_Doge_LastDoge = ret[0].action;
        this->m_Doge_StepDistance = 0;
        this->m_OnPath = false;
        if (this->m_SearchType == Vision::SEARCH_PATH_TYPE::MOVING)
            this->m_SearchType =  Vision::SEARCH_PATH_TYPE::SEARCH_180_DEGREE;
    }
    else if (this->m_Doge_LastDoge != Transmission::Action::NO_ACTION){
        if (this->m_Doge_StepDistance < Vision::STEP_DISTANCE){
            this->m_Doge_StepDistance += Vision::MOVEMENTSPEED;
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
