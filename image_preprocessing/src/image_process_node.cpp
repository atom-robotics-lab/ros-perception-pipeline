#include "image_preprocessing/image_process_node.hpp"

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


void ImagePreprocessingNode::rotateImage(cv::Mat& image) {
    double angle = static_cast<double>(rotation_angle);

    cv::Point2f center((image.cols - 1) / 2.0, (image.rows - 1) / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    rot.at<double>(0, 2) += bbox.width / 2.0 - image.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - image.rows / 2.0;

    cv::warpAffine(image, image, rot, bbox.size());
}

void ImagePreprocessingNode::convertToGrayscale(cv::Mat& image) {
    cv::Mat grayscale_image;
    cv::cvtColor(cv_ptr->image, grayscale_image, cv::COLOR_BGR2GRAY);
    // Modify the input image directly
    image = grayscale_image;
}

void ImagePreprocessingNode::resizeImage(cv::Mat& image, int width, int height) {
    cv::resize(image, image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
}

void ImagePreprocessingNode::resizeImage(cv::Mat& image, float width_multiplier, float height_multiplier) {
    int new_width = static_cast<int>(image.cols * width_multiplier);
    int new_height = static_cast<int>(image.rows * height_multiplier);
    cv::resize(image, image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
}

void ImagePreprocessingNode::flipImage(cv::Mat& image, int type){
    // 0 for x-axis
    // 1 for y-axis
    // -1 for z-axis

    cv::Mat final_image;
    cv::flip(image, final_image, type);
    image = final_image;
}

void ImagePreprocessingNode::gaussianBlurImage(cv::Mat& image, int kernel_size, int sigma){
    //gaussian blur
    cv::Mat blurred_image;
    cv::GaussianBlur(image, blurred_image, cv::Size(kernel_size, kernel_size), sigma, sigma);
    image = blurred_image;
}

void ImagePreprocessingNode::sharpenImage(cv::Mat& image, int amount ){
    float boundary = (1-amount)/4;
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, boundary, 0, boundary, amount, boundary, 0, boundary, 0);
    cv::Mat result;
    
    cv::filter2D(image, result, -1, kernel);
    image = result;
}

void ImagePreprocessingNode::thresholdImage(cv::Mat& image, float value){
    cv::Mat thresholded_image;
    
    cv::threshold(image, thresholded_image, value, 255.0, cv::THRESH_BINARY);

    image = thresholded_image;
}


void ImagePreprocessingNode::publishImage(cv::Mat& image) {
    cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", image);
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_image.toImageMsg();
    imagepublisher->publish(*output_msg);
}


void ImagePreprocessingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Perform each processing step on a copy of the image
    cv::Mat processed_image = cv_ptr->image.clone();  // Make a copy of the original image

    // // Rotate the image
    // rotateImage(processed_image);

    // // Convert to grayscale
    convertToGrayscale(processed_image);

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


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePreprocessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
