#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <string>

void imgCallback(const gz::msgs::Image &msg)
{
    cv::Mat img_raw(msg.height(), msg.width(), CV_8UC3);
    memcpy(img_raw.data, msg.data().data(), msg.data().size());

    cv::Mat img_gray;
    cv::cvtColor(img_raw, img_gray, cv::COLOR_RGB2GRAY);

    cv::Mat img_resized;
    cv::resize(img_gray, img_resized, cv::Size(640, 480));

    cv::imshow("Gazebo Preview", img_resized);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    if (argc < 2)
        std::cout << "USAGE: gz_preview_mono GAZEBO_TOPIC" << std::endl;

    const std::string GAZEBO_TOPIC = argv[1];

    std::cout << "Starting Gazebo Preview - Monocular" << std::endl;

    gz::transport::Node node;
    if (!node.Subscribe(GAZEBO_TOPIC, imgCallback))
    {
        std::cerr << "Failed to subscribe to topic: " << GAZEBO_TOPIC << std::endl;
        return 1;
    }

    std::cout << "Subsribed to topic: " << GAZEBO_TOPIC << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
