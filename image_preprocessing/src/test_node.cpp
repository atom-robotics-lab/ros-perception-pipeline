#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher_node") {

        imagesubscription = create_subscription<sensor_msgs::msg::Image>(
            "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                imageCallback(msg);
            });

        imagepublisher = create_publisher<sensor_msgs::msg::Image>("img_pub", 10);

        publishtimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        });
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        imageTranspose(cv_ptr->image);              
    }

    void imageTranspose(cv::Mat& image) {
        cv::transpose(image, image);
        publishImage(image);
    }

    void publishImage(cv::Mat& image) {
        output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        imagepublisher->publish(*output_msg.get());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imagesubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagepublisher;
    rclcpp::TimerBase::SharedPtr publishtimer;
    sensor_msgs::msg::Image::SharedPtr output_msg;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
