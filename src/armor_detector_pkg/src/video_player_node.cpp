#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/header.hpp"
#include "interfaces_pkg/msg/point2f.hpp"

class video_player_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<interfaces_pkg::msg::Point2f>::SharedPtr subscriber_point2f;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_image;
    rclcpp::Subscription<interfaces_pkg::msg::Point2f>::SharedPtr subscriber_prediction;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::Point2f topLeft, bottomRight;
    float prediction_x, prediction_y;
    int i = 0;
public:
    video_player_node(const std::string & node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("final_result", 10);
        RCLCPP_INFO(get_logger(), "video_player_node launched");
        subscriber_point2f = this->create_subscription<interfaces_pkg::msg::Point2f>("Point2f", 10, [&](const interfaces_pkg::msg::Point2f::SharedPtr msg) ->void {
            this->topLeft.x = msg->topleft_x;
            this->topLeft.y = msg->topleft_y;
            this->bottomRight.x = msg->bottomright_x;
            this->bottomRight.y = msg->bottomright_y;
            this->i = msg->i;
            std::cout<<"("<<topLeft.x<<","<<topLeft.y<<")"<<"  "<<"("<<bottomRight.x<<","<<bottomRight.y<<")"<<std::endl;
        });
        subscriber_prediction = this->create_subscription<interfaces_pkg::msg::Point2f>("prediction", 10, [&](const interfaces_pkg::msg::Point2f::SharedPtr msg) ->void {
            prediction_x = ((topLeft.x + bottomRight.x) / 2) + msg->topleft_x;
            prediction_y = ((topLeft.y + bottomRight.y) / 2) + msg->topleft_y;
        });
        subscriber_image = this->create_subscription<sensor_msgs::msg::Image>("image", 10, [&](const sensor_msgs::msg::Image::SharedPtr msg) ->void {
            cv_bridge::CvImagePtr framePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = framePtr->image;
            cv::rectangle(frame, topLeft, bottomRight, cv::Scalar(0, 255, 0), 2);
			cv::putText(frame, std::to_string(i+1), topLeft, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point2f(prediction_x, prediction_y), 5, cv::Scalar(0, 0, 255), cv::FILLED);
            auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            this->publisher_->publish(*msg_);
        });
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<video_player_node>("video_player_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}