#include "image_preprocessing/image_process_node.hpp"
#include "image_preprocessing/image_process_function.cpp"


ImagePreprocessingNode::ImagePreprocessingNode() : Node("image_preprocessing_node") {
    // Parameter declaration
    this->declare_parameter("rotation_angle", 0);
    //this->declare_parameter("resizeFlag", false);
    this->declare_parameter("graysizeFlag", false);
    this->declare_parameter("kernel_size", 0);
    this->declare_parameter("sigma", 1);
    this->declare_parameter("amount", 1);
    this->declare_parameter("value", 255.0);
    this->declare_parameter("type", 0);
    this->declare_parameter("width_multiplier", 1);
    this->declare_parameter("height_multiplier", 1);
    this->declare_parameter("width", 0);
    this->declare_parameter("height", 0);

    imagesubscription = create_subscription<sensor_msgs::msg::Image>(
        "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            imageCallback(msg);
        });

    imagepublisher = create_publisher<sensor_msgs::msg::Image>("img_pub", 10);

    publishtimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        loadWaypoints();
    });
}

void ImagePreprocessingNode::loadWaypoints() {
    rotation_angle = this->get_parameter("rotation_angle").as_int();

    //rest of params
    //resizeFlag = this->get_parameter("resizeFlag").as_bool();
    grayscaleFlag = this->get_parameter("graysizeFlag").as_bool();
    kernel_size = this->get_parameter("kernal_size").as_int();
    sigma = this->get_parameter("sigma").as_int();
    type = this->get_parameter("type").as_int();
    value = this->get_parameter("value").as_double();
    amount = this->get_parameter("amount").as_int();
    width_multiplier = this->get_parameter("width_multiplier").as_int();
    height_multiplier = this->get_parameter("height_multiplier").as_int();
    width = this->get_parameter("width").as_int();
    height = this->get_parameter("height").as_int();







}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePreprocessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
