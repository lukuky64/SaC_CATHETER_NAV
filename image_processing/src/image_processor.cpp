#include "image_processor.h"

// Constructor
ImageProcessor::ImageProcessor(ros::NodeHandle& nh) : nh_(nh), AD_params({0.1, 2000.0, 4}), 
resize_(128), catheter_size(40)
{
    // Initialize the subscriber (!!!figure out what we want the queue size to be)
    calculateMasks(784,784);
    image_sub_ = nh_.subscribe("raw_image_topic", 10, &ImageProcessor::imageCallback, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("processed_image", 10, true);
}   

// Destructor
ImageProcessor::~ImageProcessor()
{
    // Destructor code
}

// need to fix error caused by: input_image.copyTo(output_image, mask_combined)
cv::Mat ImageProcessor::isolateGray(cv::Mat input_image)
{
    std::vector<cv::Mat> channels;
    cv::split(input_image, channels);

    // Pre-allocate diff matrices with the same size and type as the channels
    cv::Mat diff_rg = cv::Mat::zeros(channels[0].size(), channels[0].type());
    cv::Mat diff_gb = cv::Mat::zeros(channels[0].size(), channels[0].type());
    cv::Mat diff_br = cv::Mat::zeros(channels[0].size(), channels[0].type());

    // Compute absolute differences between channels
    cv::absdiff(channels[0], channels[1], diff_gb);
    cv::absdiff(channels[1], channels[2], diff_rg);
    cv::absdiff(channels[2], channels[0], diff_br);

    // Identify where the absolute differences are smaller than a threshold
    cv::Mat mask_gb, mask_rg, mask_br;
    cv::inRange(diff_gb, 0, 10, mask_gb);
    cv::inRange(diff_rg, 0, 10, mask_rg);
    cv::inRange(diff_br, 0, 10, mask_br);

    // Combine masks
    cv::Mat mask_combined = mask_gb & mask_rg & mask_br;

    // Use mask to set corresponding pixels in output image
    cv::Mat output_image = cv::Mat::zeros(input_image.size(), input_image.type());
    input_image.copyTo(output_image, mask_combined);

    return output_image;
}

// Callback function for image subscriber, currently converting bgr to grescale, change this to suit to suit ultrasonic data
void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }

    // Perform image processing on cv_ptr->image using OpenCV
    cv::Mat raw_image = cv_ptr->image;

    // Apply Gaussian blur, creating a 30x30 kernal size with SD set to 10 for x and y
    cv::Mat blurred_image;
    cv::GaussianBlur(raw_image, blurred_image, cv::Size(29, 29), 10, 10);

    // Convert to grayscale
    cv::Mat gray_image;
    // gray_image = ImageProcessor::isolateGray(raw_image);
    cv::cvtColor(blurred_image, gray_image, cv::COLOR_BGR2GRAY);

    applyMask(gray_image);

    // Apply threshold to convert the image to black and white
    // cv::Mat bw_image;
    // double threshold_value = 85;  // Set threshold value as needed
    // cv::threshold(gray_image, bw_image, threshold_value, 255, cv::THRESH_BINARY);
    
    cv::Mat img_resized;
    cv::resize(gray_image, img_resized, cv::Size(resize_, resize_));

    // Convert the OpenCV image back to a ROS message
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(cv_ptr->header, "mono8", img_resized).toImageMsg();

    // Publish the message using your ROS publisher
    image_pub_.publish(output_msg);
}


void ImageProcessor::calculateMasks(int rows, int cols) {
    frame_size = cols / 2;
    int centreRow = rows / 2;
    int centreCol = cols / 2;

    mask = cv::Mat::zeros(rows, cols, CV_8U);
    maskLarge = cv::Mat::zeros(rows, cols, CV_8U);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float distanceFromCentre = std::sqrt((x - centreCol) * (x - centreCol) + (y - centreRow) * (y - centreRow));
            if (distanceFromCentre <= catheter_size) {
                mask.at<uchar>(y, x) = 1;
            }
            if (distanceFromCentre <= frame_size) {
                maskLarge.at<uchar>(y, x) = 1;
            }
        }
    }

    cv::Mat allOnes = cv::Mat::ones(rows, cols, CV_8U);
    negativeMask = ~maskLarge & allOnes;
}

void ImageProcessor::applyMask(cv::Mat &image_) {
    // Check for 1-channel (grayscale) image
    if (image_.channels() != 1) {
        std::cerr << "The input image must be a 1-channel (grayscale) image." << std::endl;
        return;
    }

    for (int y = 0; y < image_.rows; ++y) {
        for (int x = 0; x < image_.cols; ++x) {
            if (mask.at<uchar>(y, x)) {
                image_.at<uchar>(y, x) = 0;
            }
            if (negativeMask.at<uchar>(y, x)) {
                image_.at<uchar>(y, x) = 0;
            }
        }
    }
}


// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    ImageProcessor ip(nh);

    ros::spin();
    return 0;
}
