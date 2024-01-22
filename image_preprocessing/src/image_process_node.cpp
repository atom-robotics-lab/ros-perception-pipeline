#include <image_preprocessing/image_process_node.hpp>
#include "pre_processing_functions.cpp"


ImagePreprocessingNode::ImagePreprocessingNode() : Node("image_preprocessing_node")
{
    // Parameter declaration
    this->declare_parameter("rotation_angle", 0);
    //this->declare_parameter("resizeFlag", false);
    this->declare_parameter("grayscaleFlag", false);
    this->declare_parameter("kernel_size", 0);
    this->declare_parameter("sigma", 1);
    this->declare_parameter("amount", 1);
    this->declare_parameter("value", 255.0);
    this->declare_parameter("type", 0);
    this->declare_parameter("width_multiplier", 1);
    this->declare_parameter("height_multiplier", 1);
    this->declare_parameter("width", 0);
    this->declare_parameter("height", 0);

    image_subscription = create_subscription<sensor_msgs::msg::Image>(
        "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            imageCallback(msg);
        });

    image_publisher = create_publisher<sensor_msgs::msg::Image>("/img_pub", 10);

    // publishtimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    //     loadWaypoints();
    // });
}

void ImagePreprocessingNode::publishImage(cv::Mat& image)
{
    cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", image);
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_image.toImageMsg();
    image_publisher->publish(*output_msg);
}

void ImagePreprocessingNode::loadWaypoints()
{
    // Load rotation angle parameter (in degrees)
    rotation_angle = this->get_parameter("rotation_angle").as_int();

    // Load grayscale flag parameter (true to convert image to grayscale, false otherwise)
    grayscaleFlag = this->get_parameter("grayscaleFlag").as_bool();

    // Load kernel size parameter for Gaussian blur (larger values result in more pronounced blur)
    kernel_size = this->get_parameter("kernel_size").as_int();

    // Load sigma parameter for Gaussian blur (controls the smoothing effect, larger values for more smoothing)
    sigma = this->get_parameter("sigma").as_int();

    // Load type parameter for image flipping (0: flip around x-axis, 1: flip around y-axis, -1: flip around both axes)
    type = this->get_parameter("type").as_int();

    // Load threshold value parameter for image binarization (values below this threshold become black, above become white)
    value = this->get_parameter("value").as_double();

    // Load amount parameter for image sharpening (larger values result in a more pronounced sharpening effect)
    amount = this->get_parameter("amount").as_int();

    // Load width multiplier parameter for image resizing
    width_multiplier = this->get_parameter("width_multiplier").as_int();

    // Load height multiplier parameter for image resizing
    height_multiplier = this->get_parameter("height_multiplier").as_int();

    // Load width parameter for image resizing (if width_multiplier is not used)
    width = this->get_parameter("width").as_int();

    // Load height parameter for image resizing (if height_multiplier is not used)
    height = this->get_parameter("height").as_int();
}

void ImagePreprocessingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
{

    loadWaypoints();

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Perform each processing step on a copy of the image
    cv::Mat processed_image = cv_ptr->image.clone();  // Make a copy of the original image

    // Rotate the image
    rotateImage(processed_image, rotation_angle);
    std::cout<<"rotate by "<<rotation_angle<<std::endl;

    // Convert to grayscale
    // convertToGrayscale(processed_image);

    // // Resize by 2x
    // resizeImage(processed_image, cv_ptr->image.cols * 2, cv_ptr->image.rows * 2);

    // // Flip horizontally
    // flipImage(processed_image, 0);

    // // Flip vertically
    // flipImage(processed_image, 1);

    // // Apply Gaussian blur
    // gaussianBlurImage(processed_image, 5, 1);

    // // Sharpen the image
    // sharpenImage(processed_image, 1);

    // // Apply thresholding
    // thresholdImage(processed_image, 128.0);

    // Publish the processed image
    publishImage(processed_image);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto ip_node = std::make_shared<ImagePreprocessingNode>();

    rclcpp::spin(ip_node);
    rclcpp::shutdown();
    return 0;
}
