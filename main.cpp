#include <boost/asio.hpp>

#include <iostream>
#include <thread>
#include <chrono>

#include "cv.h"

std::string getSendString(Vision::Action action){
#define CASE(str) \
    case Vision::Action::str:\
    return #str; \
    break;

    switch (action){
    CASE(NO_ACTION)
    CASE(CLOSE_CONNECTION) CASE(DISCONNECTED)
    CASE(INC_SPEED) CASE(DEC_SPEED)
    CASE(MOVE_FORWARD) CASE(MOVE_LEFT) CASE(MOVE_RIGHT) CASE(MOVE_BACKWARD)
    CASE(ROTATE_LEFT) CASE(ROTATE_RIGHT)
    CASE(HEAD_LEFT) CASE(HEAD_RIGHT) CASE(HEAD_UP) CASE(HEAD_DOWN)
    default:
        return "NO_ACTION";
    }
    #undef CASE
}

int main(int argc, char* argv[])
{
    constexpr int SIZE_X = 256;
    constexpr int SIZE_Y = 256;
    constexpr int CHANNEL_COUNT = 3;

    using namespace std::chrono_literals;
    cv::Mat center(SIZE_X, SIZE_Y, CV_8UC3);
    cv::Mat right(SIZE_X, SIZE_Y, CV_8UC3);
    cv::Mat left(SIZE_X, SIZE_Y, CV_8UC3);
    cv::namedWindow("Center");
    cv::namedWindow("Right");
    cv::namedWindow("Left");
    Vision vision;

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

        const int data_size = SIZE_X * SIZE_Y * 3;
        char* data = new char[data_size];
        std::string last = "MOVE_FORWARD";

        auto receiveImage = [&data, &socket, data_size, &ec](cv::Mat& img){
            int count = 0;
            do {
                count += socket.receive(boost::asio::buffer(&data[count], data_size - count), 0, ec);
                if (ec)
                    throw std::runtime_error("connection lost");
            } while (count != data_size);

            int dataIter = 0;
            for (int y = 0; y < img.rows; ++y){
            for (int x = 0; x < img.cols; ++x){
                img.at<cv::Vec3b>(y, x)[0] = data[dataIter++];
                img.at<cv::Vec3b>(y, x)[1] = data[dataIter++];
                img.at<cv::Vec3b>(y, x)[2] = data[dataIter++];
            }
            }
        };
        while (socket.is_open()) {
            receiveImage(left);
            receiveImage(center);
            receiveImage(right);
            cv::imshow("Left", left);
            cv::imshow("Center", center);
            cv::imshow("Right", right);
            cv::waitKey(10);

            Vision::Action action = vision.getAction(left, center, right);
            std::string toSend = getSendString(action);
            socket.send(boost::asio::buffer(toSend.data(), toSend.size()), 0, ec);
            if (ec)
                throw std::runtime_error("connection lost");
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
