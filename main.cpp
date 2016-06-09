
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
    constexpr int IMAGE_SIZE_X = 256;
    constexpr int IMAGE_SIZE_Y = 256;
    constexpr int IMAGE_BYTE_SIZE = IMAGE_SIZE_X * IMAGE_SIZE_Y;

    using namespace std::chrono_literals;
    cv::Mat center(IMAGE_SIZE_Y, IMAGE_SIZE_X, CV_8UC1);
    Vision vision;

#ifdef NETWORK
    try {
        const char* port = "11000";
        const char* addr = "192.168.2.193";
        //const char* addr = "141.100.5";

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


        cv::Mat left (256, 256, CV_8UC1);
        cv::Mat right(256, 256, CV_8UC1);
        int16_t pos_x, pos_y, rot;

        char* data = new char[IMAGE_BYTE_SIZE];
        auto receiveImage = [=, &data, &socket, &ec](cv::Mat& img){
            constexpr int reciveSize = IMAGE_SIZE_X * IMAGE_SIZE_Y;
            int count = 0;
            do {
                count += socket.receive(boost::asio::buffer(&data[count], reciveSize - count), 0, ec);
                if (ec)
                    throw std::runtime_error("connection lost");
            } while (count != reciveSize);

            int dataIter = 0;
            for (int y = 0; y < img.rows; ++y){
            for (int x = 0; x < img.cols; ++x){
                img.at<uchar>(y, x) = data[dataIter++];
            }
            }
        };
        auto receivePosition = [&pos_x, &pos_y, &rot, &data, &socket, &ec](){
            constexpr int reciveSize = sizeof(pos_x) + sizeof(pos_y) + sizeof(rot);
            int count = 0;
            do {
                count += socket.receive(boost::asio::buffer(&data[count], reciveSize - count), 0, ec);
                if (ec)
                    throw std::runtime_error("connection lost");
            } while (count != reciveSize);
            pos_x = int16_t(data[1] << 8) + data[0];
            pos_y = int16_t(data[3] << 8) + data[2];
            rot   = int16_t(data[5] << 8) + data[4];
        };

        while (socket.is_open()) {
            // receive aibo position
            receivePosition();

            // receive images
            receiveImage(center);
            receiveImage(left);
            receiveImage(right);

            std::vector<Transmission> actions = vision.getAction(pos_x, pos_y, rot, left, center, right);
            size_t sendSize = actions.size() * Transmission::WRITE_SIZE + 1;
            std::string toSend(sendSize, '\0');
            toSend[0] = char(sendSize);
            for (size_t i = 0; i < actions.size(); ++i)
                actions[i].writeData(&toSend[0], i * Transmission::WRITE_SIZE + 1);
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

