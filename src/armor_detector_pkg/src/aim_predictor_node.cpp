#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "interfaces_pkg/msg/point2f.hpp"
#include <fstream>


class aim_predictor_node : public rclcpp::Node
{
private:
    rclcpp::Subscription<interfaces_pkg::msg::Point2f>::SharedPtr subscriber_;
    rclcpp::Publisher<interfaces_pkg::msg::Point2f>::SharedPtr publisher_;
    double fx, fy, cx, cy, speed_x, speed_y, journey_x, journey_y, prediction_x, prediction_y;
    float side, distance, time_gap;
    float bullet_speed = 15;
    float real_side = 1.25;
    cv::Point2f center;
    std::queue<cv::Point2f> q;
    std::queue<rclcpp::Time> q_time;
    int direction_x, direction_y;

    void loadCameraParameters()
    {
        fx = 1795.48075;
        fy = 1788.97397;
        cx = 719.15967;
        cy = 554.12545;
    }

public:
    aim_predictor_node(const std::string &node_name) : Node(node_name)
    {
        loadCameraParameters();
        publisher_ = this->create_publisher<interfaces_pkg::msg::Point2f>("prediction", 10);
        subscriber_ = this->create_subscription<interfaces_pkg::msg::Point2f>("Point2f", 10, [&](const interfaces_pkg::msg::Point2f::SharedPtr msg) ->void {
            center = cv::Point2f(msg->topleft_x + msg->bottomright_x , msg->topleft_y + msg->bottomright_y);
            side = std::fabs(msg->bottomright_x - msg->topleft_x);
            distance =  (real_side / side) * ((fx + fy) / 2);
            q.push(center);
            q_time.push(this->get_clock()->now());
            if(!q.empty())
            {
                if(q.size() == 2 && q_time.size() == 2)
                {
                    time_gap = (q_time.back().seconds() - q_time.front().seconds())*10;
                    journey_x = (q.back().x - q.front().x);
                    speed_x = fabs(journey_x / time_gap);
                    if (journey_x >= 0)
                    {
                        direction_x = 1;
                    }
                    else
                    {
                        direction_x = -1;
                    }


                    journey_y = (q.back().y - q.front().y);
                    speed_y = fabs(journey_y / time_gap);
                    if (journey_y >= 0)
                    {
                        direction_y = 1;
                    }
                    else
                    {
                        direction_y = -1;
                    }
                    q.pop();
                    q_time.pop();
                }
                else if(q.size() > 2 && q_time.size() > 2)
                {
                    q.pop();
                    q_time.pop();
                }
            }
            prediction_x = direction_x * speed_x * (distance / bullet_speed);
            prediction_y = direction_y * speed_y * (distance / bullet_speed);
            auto msg_pre = interfaces_pkg::msg::Point2f();
            msg_pre.topleft_x = prediction_x;
            msg_pre.topleft_y = prediction_y;
            this->publisher_->publish(msg_pre);
            RCLCPP_INFO(get_logger(), "%f, %f", prediction_x, prediction_y);
            RCLCPP_INFO(get_logger(), "%f, %f", journey_x, journey_y);
        });
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<aim_predictor_node>("aim_predictor_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}