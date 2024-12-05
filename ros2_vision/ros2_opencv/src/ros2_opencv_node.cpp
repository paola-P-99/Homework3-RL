#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>  // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp>  // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // OpenCV header for image processing

using namespace std::chrono_literals;

class BlobDetectionNode : public rclcpp::Node {
public:
  BlobDetectionNode() : Node("blob_detection_node") {
    // Create a subscriber to the camera image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "videocamera", 10, std::bind(&BlobDetectionNode::image_callback, this, std::placeholders::_1));

    // Create a publisher for the processed image
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
  }

private:
  // Callback to process the incoming image from the camera
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
      return;
    }

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the range of blue color in HSV (You can adjust these values)
    cv::Scalar lower_blue(90, 100, 50);  // Lower bound for blue color in HSV
    cv::Scalar upper_blue(150, 255, 255);  // Upper bound for blue color in HSV

    // Create a mask to isolate the blue regions in the image
    cv::Mat blue_mask;
    cv::inRange(hsv_image, lower_blue, upper_blue, blue_mask);

    // Apply Gaussian blur to reduce noise in the mask
    cv::Mat blurred_mask;
    cv::GaussianBlur(blue_mask, blurred_mask, cv::Size(15, 15), 0);

    // Detect circles in the mask using HoughCircles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blurred_mask, circles, cv::HOUGH_GRADIENT, 1, 30, 50, 30, 10, 100); // Adjust parameters as needed

    // Draw the circles on the original image (red color for circles)
    cv::Mat output_image = cv_ptr->image.clone();  // Clone the original image for drawing circles
    
    if (circles.size() > 0) {
        std::cout << "[INFO] Circle Detected!" << std::endl;
        
        for (size_t i = 0; i < circles.size(); i++) {
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); // Circle center
          int radius = cvRound(circles[i][2]); // Circle radius

          // Draw the circle outline (green) with a thinner border
          cv::circle(output_image, center, radius, cv::Scalar(0, 255, 0), 2); 
        }
    }
    else {
          std::cout << "[INFO] Circle NOT Detected!" << std::endl;
    }

      // Mostra l'immagine con il contorno disegnato
    cv::imshow("Output Image", output_image);
    cv::waitKey(1); // Necessario per aggiornare la finestra

    // Convert back to a ROS image message and publish it
    sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output_image).toImageMsg();
    image_pub_->publish(*processed_msg);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create a ROS2 node
  auto node = std::make_shared<BlobDetectionNode>();
  
  // Process ROS2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

