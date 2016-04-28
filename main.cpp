#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
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

		const int data_size = 256 * 256 * 3;
		char* data = new char[data_size];
		while (socket.is_open()) {
			std::cout << "worked" << std::endl;
			size_t ret = socket.receive(boost::asio::buffer(data, data_size), 0, ec);
			if (ec)
				throw std::runtime_error("connection lost");

			printf("Returned size: %d\n\n", ret);
			const char buf[] = "MOVE_FORWARD";
			socket.send(boost::asio::buffer(buf, sizeof(buf) - 1), 0, ec);
			if (ec)
				throw std::runtime_error("connection lost");

			std::this_thread::sleep_for(100ms);
		}
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
	return 0;
}
