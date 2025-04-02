#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <mutex>
#include <atomic>

ORB_SLAM3::System *pSLAM = nullptr;
std::mutex img_mutex;
cv::Mat latest_img;
double latest_timestamp = 0;
bool has_new_frame = false;
std::atomic<bool> stopSLAM(false);

void listenForStopCommand() {
    std::string input;
    while (!stopSLAM) {
        std::cin >> input;
        if (input == "stop") {
            stopSLAM = true;
            std::cout << "Stopping ORB-SLAM3..." << std::endl;
        }
    }
}

void imgCallback(const gz::msgs::Image &msg)
{
    cv::Mat img_raw(msg.height(), msg.width(), CV_8UC3);
    memcpy(img_raw.data, msg.data().data(), msg.data().size());

    cv::Mat img_gray;
    cv::cvtColor(img_raw, img_gray, cv::COLOR_RGB2GRAY);

    std::lock_guard<std::mutex> lock(img_mutex);
    latest_img = img_gray.clone();
    latest_timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
    has_new_frame = true;
}

int main(int argc, char **argv)
{
    if (argc < 4)
        std::cout << "USAGE: gz_video_mono PATH_TO_VOCABULARY PATH_TO_CONFIG GAZEBO_TOPIC" << std::endl;

    const std::string PATH_TO_VOCAB = argv[1];
    const std::string PATH_TO_CONFIG = argv[2];
    const std::string GAZEBO_TOPIC = argv[3];

    ORB_SLAM3::System SLAM(PATH_TO_VOCAB, PATH_TO_CONFIG, ORB_SLAM3::System::MONOCULAR, true);
    pSLAM = &SLAM;

    gz::transport::Node node;
    if (!node.Subscribe(GAZEBO_TOPIC, imgCallback))
    {
        std::cerr << "Failed to subscribe to topic: " << GAZEBO_TOPIC << std::endl;
        return 1;
    }
    std::cout << "Subscribed to topic: " << GAZEBO_TOPIC << std::endl;

    std::thread stopListener(listenForStopCommand);

    while (!stopSLAM)
    {
        cv::Mat img;
        double timestamp = 0;

        {
            std::lock_guard<std::mutex> lock(img_mutex);
            if (!has_new_frame)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            img = latest_img.clone();
            timestamp = latest_timestamp;
            has_new_frame = false;
        }

        cv::imshow("Gazebo Preview", img);
        SLAM.TrackMonocular(img, timestamp);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_Gazebo.txt");
    std::cout << "Keyframe trajectory saved to KeyFrameTrajectory_Gazebo.txt" << std::endl;

    stopListener.join();

    return 0;
}