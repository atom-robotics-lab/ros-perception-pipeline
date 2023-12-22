#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePreprocessingNode : public rclcpp::Node {
public:
    ImagePreprocessingNode();

private:
    void loadWaypoints();
    
    void rotateImage(cv::Mat& image);
    void convertToGrayscale(cv::Mat& image);
    void resizeImage(cv::Mat& image, int width, int height);
    void resizeImage(cv::Mat& image, float width_multiplier, float height_multiplier);
    void flipImage(cv::Mat& image, int type);
    void gaussianBlurImage(cv::Mat& image, int kernel_size, int sigma);
    void sharpenImage(cv::Mat& image, int amount );
    void thresholdImage(cv::Mat& image, float value);

    void publishImage(cv::Mat& image);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imagesubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagepublisher;
    rclcpp::TimerBase::SharedPtr publishtimer;

    sensor_msgs::msg::Image::SharedPtr output_msg;

    cv::Mat final_image;

    //params
    int rotation_angle = 0;  // Member variable to store rotation angle
    //bool resizeFlag = false;
    bool grayscaleFlag = false;
    int kernel_size = 0;
    int amount = 0;
    int value = 255.0 ;
    int sigma = 1;
    int type = 0;
    int width_multiplier = 1;
    int height_multiplier = 1;
    int height = 0;
    int width = 0;
    
};
