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

    // Parameters for image processing functions
    int rotation_angle = 0;  // Rotation angle in degrees
    bool resizeFlag = false   // Flag for resizing
    bool grayscaleFlag = false;  // Flag to enable grayscale conversion
    int kernel_size = 0;  // Kernel size for Gaussian blur
    int amount = 0;  // Amount for image sharpening
    float value = 255.0;  // Threshold value
    int sigma = 1;  // Smoothing amount for Gaussian blur
    int type = 0;  // Type parameter for image flipping (0: x-axis, 1: y-axis, -1: both)
    int width_multiplier = 1;  // Width multiplier for image resizing
    int height_multiplier = 1;  // Height multiplier for image resizing
    int height = 0;  // Height for image resizing
    int width = 0;  // Width for image resizing
    
};
