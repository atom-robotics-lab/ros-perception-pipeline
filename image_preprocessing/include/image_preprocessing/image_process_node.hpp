#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode();

private:
    void loadWaypoints();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void rotateImage(cv::Mat& image);
    void publishImage(cv::Mat& image);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imagesubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagepublisher;
    rclcpp::TimerBase::SharedPtr publishtimer;

    sensor_msgs::msg::Image::SharedPtr output_msg;

    int rotation_angle;  // Member variable to store rotation angle
};
