#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePreprocessingNode : public rclcpp::Node
{
public:
    ImagePreprocessingNode();

private:
    void loadWaypoints();

    void publishImage(cv::Mat& image);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;

    sensor_msgs::msg::Image::SharedPtr output_msg;

    cv::Mat final_image;

    // Parameters for image processing functions

    // Rotation angle in degrees
    int rotation_angle = 0;

    // Flag for resizing  
    bool resizeFlag = false; 

    // Flag to enable grayscale conversion  
    bool grayscaleFlag = false;  

    // Kernel size for Gaussian blur
    int kernel_size = 0;  

    // Amount for image sharpening
    int amount = 0;  

    // Threshold value
    float value = 255.0;

     // Smoothing amount for Gaussian blur 
    int sigma = 1; 

    // Type parameter for image flipping (0: x-axis, 1: y-axis, -1: both)
    int type = 0;  

    // Width multiplier for image resizing
    int width_multiplier = 1;  

    // Height multiplier for image resizing
    int height_multiplier = 1;  

    // Height for image resizing
    int height = 0; 

    // Width for image resizing   
    int width = 0;    
};
