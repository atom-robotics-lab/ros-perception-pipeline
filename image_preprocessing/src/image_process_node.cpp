#include "image_preprocessing/image_process_node.hpp"


ImagePublisherNode::ImagePublisherNode() : Node("image_publisher_node") {
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

void ImagePublisherNode::loadWaypoints() {
    rotation_angle = this->get_parameter("rotation_angle").as_int();
}

void ImagePublisherNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    rotateImage(cv_ptr->image);
}

void ImagePublisherNode::rotateImage(cv::Mat& image) {
    double angle = static_cast<double>(rotation_angle);

    cv::Point2f center((image.cols - 1) / 2.0, (image.rows - 1) / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    rot.at<double>(0, 2) += bbox.width / 2.0 - image.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - image.rows / 2.0;

    cv::Mat rotated_image;
    cv::warpAffine(image, rotated_image, rot, bbox.size());

    publishImage(rotated_image);
}

void ImagePublisherNode::publishImage(cv::Mat& image) {
    cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", image);
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_image.toImageMsg();
    imagepublisher->publish(*output_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}
