#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode() : Node("image_publisher_node") {

        // Parameter declaration
        this->declare_parameter("rotation_angle", 0);
        
        imagesubscription = create_subscription<sensor_msgs::msg::Image>(
            "/color_camera/image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                imageCallback(msg);
            });

        imagepublisher = create_publisher<sensor_msgs::msg::Image>("img_pub", 10);

        publishtimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            loadWaypoints();
        });
    }

private:
    void loadWaypoints() {
        rotation_angle = this->get_parameter("rotation_angle").as_int();
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        rotateImage(cv_ptr->image);
    }

    void rotateImage(cv::Mat& image) {
        // Process image according to the rotation angle
        std::cout << rotation_angle << std::endl;
        if (rotation_angle % 90 != 0) {
            RCLCPP_ERROR(get_logger(), "Invalid rotation angle. Must be a multiple of 90 degrees.");
            return;
        }

        for (int i = 0; i < abs(rotation_angle / 90); ++i) {
            // Rotate the image by 90 degrees clockwise (or counter-clockwise)
            cv::transpose(image, image);
            cv::flip(image, image, (rotation_angle > 0) ? 1 : 0);
        }

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
    int rotation_angle;  // Member variable to store rotation angle
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}