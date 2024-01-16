#include "image_preprocessing/image_process_node.hpp"
#include "image_preprocessing/image_process_function.cpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ip_node = std::make_shared<ImagePreprocessingNode>();

    rclcpp::spin(ip_node);
    rclcpp::shutdown();
    return 0;
}
