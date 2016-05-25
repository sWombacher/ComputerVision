
#include <iostream>
#include <thread>
#include <chrono>

#include "cv.h"



#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <iostream>
#include <string>


#define NETWORK

#ifdef NETWORK
#include <boost/asio.hpp>
#endif


int main(int, char**)
{
    constexpr int CENTER_SIZE_X = 256;
    constexpr int CENTER_SIZE_Y = 256;
    constexpr int CENTER_BYTE_SIZE = CENTER_SIZE_X * CENTER_SIZE_Y * 3;

    constexpr int DISPARITY_SIZE_X = 192;
    constexpr int DISPARITY_SIZE_Y = 256;
    constexpr int DISPAIRTY_BYTE_SIZE = DISPARITY_SIZE_X * DISPARITY_SIZE_Y;

    using namespace std::chrono_literals;
    cv::Mat center(CENTER_SIZE_Y, CENTER_SIZE_X, CV_8UC3);
    cv::Mat disparity(DISPARITY_SIZE_Y, DISPARITY_SIZE_X, CV_8UC1);
    cv::namedWindow("Center");
    cv::namedWindow("Disparity");
    Vision vision;

#ifdef NETWORK
    try {
        const char* port = "11000";
        const char* addr = "192.168.2.192";

        boost::asio::io_service io_service;
        boost::asio::ip::tcp::socket socket(io_service);
        boost::system::error_code ec;

        boost::asio::ip::tcp::resolver resolver(io_service);
        auto iter = resolver.resolve({ addr , port }, ec);
        if (ec)
            throw std::runtime_error("Error: Resolver couldn't be initialized");

        io_service.run();
        socket.connect(*iter);
        if (ec)
            throw std::runtime_error("Error: Socket couldn't be opend");

        char* data = new char[(CENTER_BYTE_SIZE > DISPAIRTY_BYTE_SIZE ? CENTER_BYTE_SIZE : DISPAIRTY_BYTE_SIZE)];

        auto receiveImage = [=, &data, &socket, &ec](cv::Mat& img, const bool gray){
            const int reciveSize = gray ? DISPAIRTY_BYTE_SIZE : CENTER_BYTE_SIZE;
            int count = 0;
            do {
                count += socket.receive(boost::asio::buffer(&data[count], reciveSize - count), 0, ec);
                if (ec)
                    throw std::runtime_error("connection lost");
            } while (count != reciveSize);

            int dataIter = 0;
            for (int y = 0; y < img.rows; ++y){
            for (int x = 0; x < img.cols; ++x){
                if (gray)
                    img.at<uchar>(y, x) = data[dataIter++];
                else{
                    img.at<cv::Vec3b>(y, x)[0] = data[dataIter++];
                    img.at<cv::Vec3b>(y, x)[1] = data[dataIter++];
                    img.at<cv::Vec3b>(y, x)[2] = data[dataIter++];
                }
            }
            }
        };

        cv::Mat left (256, 256, CV_8UC3);
        cv::Mat right(256, 256, CV_8UC3);
        while (socket.is_open()) {
            receiveImage(center, false);
            receiveImage(disparity, true);
            receiveImage(left ,false);
            receiveImage(right,false);

            std::vector<Transmission> actions = vision.getAction(left, center, right);
            std::string toSend(actions.size() * Transmission::WRITE_SIZE, '\0');
            for (size_t i = 0; i < actions.size(); i += Transmission::WRITE_SIZE)
                actions[i].writeData(&toSend[0], i);
            socket.send(boost::asio::buffer(toSend.data(), toSend.size()), 0, ec);
            if (ec)
                throw std::runtime_error("connection lost");
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

#else
    int currentIdx = 0;
    while (true) {
        center = cv::imread(("center_" + std::to_string(currentIdx) + ".png").c_str());
        disparity = cv::imread(("disparity_" + std::to_string(currentIdx) + ".png").c_str());
        std::string str= getSendString(vision.getAction(center, disparity));
        printf("Current index: %d\nMovement: %s\n", currentIdx++, str.c_str());

        cv::imshow("Center", center);
        cv::imshow("disparity", disparity);
        cv::waitKey(10);
    }
#endif
    return 0;
}

