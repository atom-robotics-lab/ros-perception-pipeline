#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher_node") {
        // Create a subscription to the "/color_camera/image_raw" topic
        imagesubscription = create_subscription<sensor_msgs::msg::Image>(
            "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                // Callback function for the subscription
                publishImage(msg);
            });

        // Create a publisher for the "img_pub" topic
        imagepublisher = create_publisher<sensor_msgs::msg::Image>("img_pub", 10);

        // Set the publishing rate to 10 Hz
        publishtimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            // Timer callback for publishing at 10 Hz
            // You can perform any additional processing here if needed
        });
    }

private:
    void publishImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert sensor_msgs::Image to cv::Mat using cv_bridge
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Add text to the image
        cv::putText(cv_ptr->image, "Img Processed", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

        // Publish the modified image
        auto modified_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
        imagepublisher->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imagesubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagepublisher;
    rclcpp::TimerBase::SharedPtr publishtimer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}