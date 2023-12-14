#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher_node") {
        // Create a subscription to the "/color_camera/image_raw" topic
        imagesubscription = create_subscription<sensor_msgs::msg::Image>(
            "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                // Callback function for the subscription
                publishImage(msg);
               // std::cout<<"Publishing Image"<<endl;
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
        auto loaned_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
        imagepublisher->publish(std::move(loaned_msg));
        
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