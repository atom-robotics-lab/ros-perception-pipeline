#include <opencv2/opencv.hpp>

static void rotateImage(cv::Mat& image, int rotation_angle)
{
    double angle = static_cast<double>(rotation_angle);

    cv::Point2f center((image.cols - 1) / 2.0, (image.rows - 1) / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
    rot.at<double>(0, 2) += bbox.width / 2.0 - image.cols / 2.0;
    rot.at<double>(1, 2) += bbox.height / 2.0 - image.rows / 2.0;

    cv::warpAffine(image, image, rot, bbox.size());
}

static void convertToGrayscale(cv::Mat& image)
{
    cv::Mat grayscale_image;
    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
    // Modify the input image directly
    image = grayscale_image;
}

// static void resizeImage(cv::Mat& image, int width, int height)
// {
//     cv::resize(image, image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
// }

static void resizeImage(cv::Mat& image, float width_multiplier, float height_multiplier)
{
    int new_width = static_cast<int>(image.cols * width_multiplier);
    int new_height = static_cast<int>(image.rows * height_multiplier);

    cv::resize(image, image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
}

static void flipImage(cv::Mat& image, int type)
{
    // 0 for x-axis
    // 1 for y-axis
    // -1 for z-axis

    cv::Mat final_image;
    cv::flip(image, final_image, type);
    image = final_image;
}

static void gaussianBlurImage(cv::Mat& image, int kernel_size, int sigma)
{
    //gaussian blur
    cv::Mat blurred_image;
    cv::GaussianBlur(image, blurred_image, cv::Size(kernel_size, kernel_size), sigma, sigma);
    image = blurred_image;
}

static void sharpenImage(cv::Mat& image, int amount )
{
    float boundary = (1-amount)/4;
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, boundary, 0, boundary, amount, boundary, 0, boundary, 0);
    cv::Mat result;
    
    cv::filter2D(image, result, -1, kernel);
    image = result;
}

static void thresholdImage(cv::Mat& image, float value)
{
    cv::Mat thresholded_image;    
    cv::threshold(image, thresholded_image, value, 255.0, cv::THRESH_BINARY);
    image = thresholded_image;
}
